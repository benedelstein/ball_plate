#!/usr/bin/env python3
"""Small, dependency-free server for the wedding website."""

from __future__ import annotations

import hashlib
import hmac
import html
import json
import mimetypes
import os
import secrets
import time
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs


ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get("PORT", "8080"))
PASSWORD = os.environ.get("WEDDING_PASSWORD", "")
SIGNING_KEY = os.environ.get("COOKIE_SECRET", secrets.token_hex(32)).encode()
SESSION_COOKIE = "boda_session"
RSVP_FILE = ROOT / "data" / "rsvps.ndjson"

ALLOWED_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
    "/script.js": ("script.js", "text/javascript; charset=utf-8"),
    "/assets/wedding-hero.png": ("assets/wedding-hero.png", "image/png"),
}


def signed_session() -> str:
    issued = str(int(time.time()))
    signature = hmac.new(SIGNING_KEY, issued.encode(), hashlib.sha256).hexdigest()
    return f"{issued}.{signature}"


def valid_session(value: str | None) -> bool:
    if not value or "." not in value:
        return False
    issued, signature = value.split(".", 1)
    expected = hmac.new(SIGNING_KEY, issued.encode(), hashlib.sha256).hexdigest()
    try:
        not_expired = int(issued) > int(time.time()) - 60 * 60 * 24 * 30
    except ValueError:
        return False
    return hmac.compare_digest(signature, expected) and not_expired


def login_page(error: str = "") -> bytes:
    error_markup = f'<p class="error">{html.escape(error)}</p>' if error else ""
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="theme-color" content="#f6be2f">
  <title>Bendavid Black Boda Bonanza</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{ margin:0; min-height:100svh; display:grid; place-items:center; padding:24px;
      color:#241439; background:#f6be2f; font-family:Arial,sans-serif; }}
    main {{ width:min(100%, 680px); position:relative; padding:clamp(40px,8vw,85px);
      background:#fffaf0; border:1px solid #241439; box-shadow:14px 14px 0 #ef593e; }}
    .kicker {{ margin:0 0 22px; font-size:11px; font-weight:700; letter-spacing:.16em; text-transform:uppercase; }}
    h1 {{ margin:0; font:400 clamp(58px,12vw,105px)/.84 Georgia,serif; letter-spacing:-.055em; }}
    h1 em {{ display:block; color:#2147c7; font-weight:400; }}
    .intro {{ max-width:440px; margin:35px 0 28px; line-height:1.65; }}
    label {{ display:block; margin-bottom:9px; font-size:10px; font-weight:700; letter-spacing:.12em; text-transform:uppercase; }}
    .row {{ display:flex; gap:10px; }}
    input {{ min-width:0; flex:1; padding:15px 16px; color:#241439; background:#f5e8cf;
      border:1px solid #241439; border-radius:0; font:inherit; }}
    button {{ padding:0 22px; color:#fffaf0; background:#241439; border:1px solid #241439;
      font-size:11px; font-weight:700; letter-spacing:.1em; text-transform:uppercase; cursor:pointer; }}
    .error {{ color:#c52f28; font-size:13px; font-weight:700; }}
    .stamp {{ width:82px; height:82px; position:absolute; top:25px; right:25px; display:grid;
      place-items:center; border:1px solid #241439; border-radius:50%; font:700 10px/1.2 Arial;
      text-align:center; letter-spacing:.1em; transform:rotate(9deg); }}
    @media(max-width:560px) {{ main {{ padding:50px 24px; box-shadow:8px 8px 0 #ef593e; }}
      .stamp {{ width:65px; height:65px; top:18px; right:18px; }} .row {{ flex-direction:column; }}
      button {{ min-height:50px; }} }}
  </style>
</head>
<body>
  <main>
    <div class="stamp">08 · 02<br>· 26</div>
    <p class="kicker">Caroline + Noam · San Francisco</p>
    <h1>Boda <em>Bonanza</em></h1>
    <p class="intro">This celebration is just for invited guests. Enter the password from your invitation to come on in.</p>
    {error_markup}
    <form method="post" action="/unlock">
      <label for="password">Wedding password</label>
      <div class="row">
        <input id="password" name="password" type="password" autocomplete="current-password" required autofocus>
        <button type="submit">Enter →</button>
      </div>
    </form>
  </main>
</body>
</html>""".encode()


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

    def session_is_valid(self) -> bool:
        jar = cookies.SimpleCookie(self.headers.get("Cookie", ""))
        morsel = jar.get(SESSION_COOKIE)
        return valid_session(morsel.value if morsel else None)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/health":
            self.send_bytes(b'{"status":"ok"}', "application/json")
            return

        if PASSWORD and not self.session_is_valid():
            self.send_bytes(login_page(), "text/html; charset=utf-8", 401)
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

        if path == "/unlock":
            fields = parse_qs(body.decode(errors="replace"))
            supplied = fields.get("password", [""])[0]
            if PASSWORD and hmac.compare_digest(supplied, PASSWORD):
                self.send_response(303)
                self.send_header("Location", "/")
                self.send_header(
                    "Set-Cookie",
                    f"{SESSION_COOKIE}={signed_session()}; Path=/; HttpOnly; Secure; SameSite=Lax; Max-Age=2592000",
                )
                self.end_headers()
            else:
                self.send_bytes(login_page("That password didn’t work. Try again."), "text/html; charset=utf-8", 401)
            return

        if path == "/rsvp":
            if PASSWORD and not self.session_is_valid():
                self.send_bytes(b'{"message":"Please unlock the website first."}', "application/json", 401)
                return
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
