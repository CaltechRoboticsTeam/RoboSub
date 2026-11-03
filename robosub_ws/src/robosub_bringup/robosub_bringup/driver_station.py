"""Serve the loopback Xbox driver station and command preview."""

import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from robosub_control.utils.tuning import fixed_parameter

from robosub_bringup.operator_panel import OperatorPanel


class DriverStation(Node):
    def __init__(self):
        super().__init__("driver_station")
        self.port = fixed_parameter(self, "port", 8765)
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("port must be an integer between 1 and 65535")
        self.operator = OperatorPanel(self)


def make_handler(station, html, token, gamepad_script=""):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *args):
            pass

        def respond(self, status, value, content_type="application/json"):
            if content_type == "application/json":
                value = json.dumps(value, allow_nan=False)
            body = value.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.respond(200, html.replace("__TOKEN__", token), "text/html")
            elif self.path == "/gamepad.js":
                self.respond(200, gamepad_script, "text/javascript")
            elif self.path == "/api/gamepad":
                self.respond(200, station.operator.snapshot())
            else:
                self.respond(404, {"error": "Not found"})

        def do_POST(self):
            if self.path != "/api/gamepad":
                self.respond(404, {"error": "Not found"})
                return
            if not secrets.compare_digest(
                self.headers.get("X-Driver-Station-Token", ""), token
            ):
                self.respond(403, {"error": "Missing driver-station token"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("Expected a JSON object")
                result = station.operator.update(data)
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                self.respond(400, {"error": str(error)})
                return
            self.respond(200, result)

    return Handler


def main():
    rclpy.init()
    node = DriverStation()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    try:
        share = Path(get_package_share_directory("robosub_bringup"))
        handler = make_handler(
            node,
            (share / "web/driver_station.html").read_text(),
            secrets.token_urlsafe(32),
            (share / "web/gamepad.js").read_text(),
        )
        with ThreadingHTTPServer(("127.0.0.1", node.port), handler) as server:
            node.get_logger().info(
                f"Driver station: http://127.0.0.1:{node.port}"
            )
            server.timeout = 0.25
            while rclpy.ok():
                server.handle_request()
    except KeyboardInterrupt:
        pass
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        node.get_logger().error(
            f"Driver station port {node.port} is already in use. "
            "Stop the previous RoboSub launch, or choose a different "
            "driver_station_port if another application owns it."
        )
        return 1
    finally:
        executor.shutdown()
        worker.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
