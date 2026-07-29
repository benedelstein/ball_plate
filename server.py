#!/usr/bin/env python3
"""Small, dependency-free server for the wedding website."""

from __future__ import annotations

import json
import mimetypes
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get("PORT", "8080"))
RSVP_FILE = ROOT / "data" / "rsvps.ndjson"

ALLOWED_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
    "/script.js": ("script.js", "text/javascript; charset=utf-8"),
    "/assets/wedding-hero.png": ("assets/wedding-hero.png", "image/png"),
}


class WeddingHandler(BaseHTTPRequestHandler):
    server_version = "BodaBonanza/1.0"

    def send_bytes(self, payload: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "strict-origin-when-cross-origin")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; "
            "style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; "
            "font-src https://fonts.gstatic.com; script-src 'self'; "
            "connect-src 'self'; form-action 'self'; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/health":
            self.send_bytes(b'{"status":"ok"}', "application/json")
            return

        item = ALLOWED_FILES.get(path)
        if not item:
            self.send_bytes(b"Not found", "text/plain; charset=utf-8", 404)
            return

        relative_path, content_type = item
        file_path = ROOT / relative_path
        self.send_bytes(file_path.read_bytes(), content_type)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length", "0"))
        if length > 20_000:
            self.send_bytes(b"Request too large", "text/plain; charset=utf-8", 413)
            return
        body = self.rfile.read(length)

        if path == "/rsvp":
            content_type = self.headers.get("Content-Type", "")
            if "multipart/form-data" not in content_type:
                self.send_bytes(b'{"message":"Unsupported request."}', "application/json", 415)
                return

            # The browser uses multipart FormData. Parse only the simple text fields we accept.
            boundary_value = content_type.split("boundary=", 1)[-1].strip().strip('"')
            fields: dict[str, str] = {}
            if boundary_value:
                boundary = ("--" + boundary_value).encode()
                for part in body.split(boundary):
                    if b"\r\n\r\n" not in part:
                        continue
                    headers_blob, value = part.split(b"\r\n\r\n", 1)
                    value = value.rstrip(b"\r\n-")
                    marker = b'name="'
                    if marker not in headers_blob:
                        continue
                    name = headers_blob.split(marker, 1)[1].split(b'"', 1)[0].decode(errors="ignore")
                    if name in {"name", "attending", "dietary", "note"}:
                        fields[name] = value.decode(errors="replace")[:1000].strip()

            if not fields.get("name") or fields.get("attending") not in {
                "joyfully-accepts",
                "regretfully-declines",
            }:
                self.send_bytes(b'{"message":"Please complete the required fields."}', "application/json", 400)
                return

            RSVP_FILE.parent.mkdir(exist_ok=True)
            record = {
                "received_at": int(time.time()),
                "name": fields["name"],
                "attending": fields["attending"],
                "dietary": fields.get("dietary", ""),
                "note": fields.get("note", ""),
            }
            with RSVP_FILE.open("a", encoding="utf-8") as output:
                output.write(json.dumps(record, ensure_ascii=False) + "\n")
            payload = json.dumps({"message": "RSVP received—thank you! We can’t wait to celebrate."}).encode()
            self.send_bytes(payload, "application/json")
            return

        self.send_bytes(b"Not found", "text/plain; charset=utf-8", 404)

    def log_message(self, format: str, *args: object) -> None:
        print(f"{self.address_string()} - {format % args}")


if __name__ == "__main__":
    mimetypes.add_type("text/javascript", ".js")
    print(f"Bendavid Black Boda Bonanza is serving on 0.0.0.0:{PORT}")
    ThreadingHTTPServer(("0.0.0.0", PORT), WeddingHandler).serve_forever()
