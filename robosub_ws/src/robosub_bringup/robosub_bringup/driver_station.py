"""Serve the loopback Xbox driver station and command preview."""

import errno
import json
import secrets
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robosub_control.utils.tuning import fixed_parameter

from robosub_bringup.mac_gamepad import MacGamepad
from robosub_bringup.operator_panel import OperatorPanel


class DriverStation(Node):
    def __init__(self):
        super().__init__("driver_station")
        self.port = fixed_parameter(self, "port", 8765)
        self.operator = OperatorPanel(self)
        self.diagnostics_lock = threading.Lock()
        self.diagnostics = []
        self.diagnostics_received = None
        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.on_diagnostics,
            qos_profile_sensor_data,
        )

    def on_diagnostics(self, message):
        prefix = "sub/Diagnostics/"
        rows = [
            {
                "name": status.name.removeprefix(prefix),
                "level": status.level[0],
                "message": status.message,
                "values": {item.key: item.value for item in status.values},
            }
            for status in message.status
            if status.name.startswith(prefix)
        ]
        if rows:
            with self.diagnostics_lock:
                self.diagnostics = rows
                self.diagnostics_received = time.monotonic()

    def diagnostics_snapshot(self):
        with self.diagnostics_lock:
            age = (
                time.monotonic() - self.diagnostics_received
                if self.diagnostics_received is not None
                else None
            )
            return {"rows": self.diagnostics, "age_s": age}


def make_handler(
    station,
    html,
    token,
    gamepad_script="",
    native_gamepad=None,
    diagnostics_script="",
):
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
            elif self.path == "/diagnostics.js":
                self.respond(200, diagnostics_script, "text/javascript")
            elif self.path == "/api/diagnostics":
                self.respond(200, station.diagnostics_snapshot())
            elif self.path == "/api/gamepad":
                data = station.operator.snapshot()
                data["native_controller"] = native_gamepad is not None
                self.respond(200, data)
            elif self.path == "/api/controller":
                self.respond(
                    200,
                    native_gamepad.snapshot()
                    if native_gamepad is not None
                    else {"controller": None},
                )
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
    native_gamepad = None
    try:
        if sys.platform == "darwin" and node.operator.source == "browser":
            try:
                native_gamepad = MacGamepad()
            except (OSError, subprocess.SubprocessError) as error:
                node.get_logger().warning(f"Mac controller reader: {error}")
        share = Path(get_package_share_directory("robosub_bringup"))
        handler = make_handler(
            node,
            (share / "web/driver_station.html").read_text(),
            secrets.token_urlsafe(32),
            (share / "web/gamepad.js").read_text(),
            native_gamepad,
            diagnostics_script=(share / "web/diagnostics.js").read_text(),
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
        if native_gamepad is not None:
            native_gamepad.close()
        executor.shutdown()
        worker.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
