"""Small, locked-down static server for the Bob's Donuts site."""

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
ALLOWED_FILES = {"index.html", "styles.css", "script.js"}
ALLOWED_ASSET_PREFIX = "assets/"


class SiteHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path: str) -> str:
        clean_path = urlparse(path).path.lstrip("/") or "index.html"
        requested = (ROOT / clean_path).resolve()
        try:
            relative = requested.relative_to(ROOT).as_posix()
        except ValueError:
            return str(ROOT / "__not_found__")

        allowed = relative in ALLOWED_FILES or relative.startswith(ALLOWED_ASSET_PREFIX)
        return str(requested if allowed else ROOT / "__not_found__")

    def list_directory(self, path):
        self.send_error(404)
        return None

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "strict-origin-when-cross-origin")
        self.send_header("X-Frame-Options", "DENY")
        super().end_headers()


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8080), SiteHandler).serve_forever()
