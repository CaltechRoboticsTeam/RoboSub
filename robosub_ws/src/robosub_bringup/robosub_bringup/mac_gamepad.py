"""Read the Mac Xbox controller for Firefox's incomplete gamepad mapping."""

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import urlparse


class MacGamepad:
    def __init__(self):
        if sys.platform != "darwin":
            raise RuntimeError("The native controller reader requires macOS")
        source = Path(__file__).with_suffix(".swift")
        identity = source.read_bytes() + platform.platform().encode()
        digest = hashlib.sha256(identity).hexdigest()[:16]
        cache = Path.home() / "Library/Caches/robosub"
        cache.mkdir(parents=True, exist_ok=True)
        executable = cache / f"gamepad-{digest}"
        if not executable.exists():
            with tempfile.TemporaryDirectory(dir=cache) as directory:
                build = Path(directory) / "gamepad"
                subprocess.run(
                    ["/usr/bin/swiftc", "-O", str(source), "-o", str(build)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                build.replace(executable)
        self.lock = threading.Lock()
        self.frame = {"controller": None}
        self.received = -float("inf")
        self.process = subprocess.Popen(
            [str(executable)],
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.worker = threading.Thread(target=self._read, daemon=True)
        self.worker.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                frame = json.loads(line)
            except ValueError:
                continue
            if not isinstance(frame, dict) or "controller" not in frame:
                continue
            with self.lock:
                self.frame = frame
                self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            if time.monotonic() - self.received > 0.25:
                return {
                    "controller": None,
                    "error": self.frame.get("error", "No fresh Mac input"),
                }
            return dict(self.frame)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.worker.join(timeout=1)
        self.process.stdout.close()


def make_handler(reader):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *args):
            pass

        def do_GET(self):
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).hostname not in (
                "127.0.0.1",
                "localhost",
                "::1",
            ):
                self.send_error(403)
                return
            if self.path != "/api/controller":
                self.send_error(404)
                return
            body = json.dumps(reader.snapshot(), allow_nan=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main():
    reader = MacGamepad()
    try:
        with ThreadingHTTPServer(
            ("127.0.0.1", 8767), make_handler(reader)
        ) as server:
            print("Mac controller reader: http://127.0.0.1:8767", flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


if __name__ == "__main__":
    main()
