"""Loopback-only developer menu backed by standard ROS parameter services."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import threading

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import (
    Parameter as ParameterMessage,
)
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
)
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    ListParameters,
    SetParametersAtomically,
)
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from robosub_bringup.operator_panel import OperatorPanel


class ParameterBrowser(Node):
    """Service requests wait in HTTP threads, never in a control callback."""

    def __init__(self):
        super().__init__("tuning_menu")
        self.port = self.declare_parameter(
            "port", 8765, ParameterDescriptor(read_only=True)
        ).value
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("port must be an integer between 1 and 65535")
        self.operator = OperatorPanel(self)
        self.parameter_clients = {}
        self.client_lock = threading.Lock()

    def request(self, node, kind, suffix, request):
        key = (node, suffix)
        with self.client_lock:
            if key not in self.parameter_clients:
                self.parameter_clients[key] = self.create_client(
                    kind, f"{node}/{suffix}"
                )
            client = self.parameter_clients[key]
        if not client.wait_for_service(timeout_sec=0.25):
            raise TimeoutError(f"Parameter service unavailable: {node}")
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=3):
            future.cancel()
            raise TimeoutError(f"Parameter request timed out: {node}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"Parameter request failed: {node}")
        return result

    def inspect_node(self, node):
        names = self.request(
            node, ListParameters, "list_parameters", ListParameters.Request()
        ).result.names
        if "tuning_enabled" not in names:
            return None
        enabled = self.request(
            node,
            GetParameters,
            "get_parameters",
            GetParameters.Request(names=["tuning_enabled"]),
        ).values
        if not enabled or enabled[0].type != ParameterType.PARAMETER_BOOL:
            return None
        if not enabled[0].bool_value:
            return None
        descriptors = self.request(
            node,
            DescribeParameters,
            "describe_parameters",
            DescribeParameters.Request(names=names),
        ).descriptors
        parameters = []
        for name, descriptor in zip(names, descriptors, strict=True):
            try:
                metadata = json.loads(descriptor.additional_constraints)
            except (ValueError, TypeError):
                continue
            if not isinstance(metadata, dict) or not metadata.get(
                "robosub_tunable"
            ):
                continue
            parameters.append(
                {
                    "name": name,
                    "default": metadata["default"],
                    "read_only": descriptor.read_only,
                    "description": descriptor.description,
                    "type": descriptor.type,
                }
            )
        if not parameters:
            return None
        parameters.sort(key=lambda entry: entry["name"])
        values = self.request(
            node,
            GetParameters,
            "get_parameters",
            GetParameters.Request(names=[p["name"] for p in parameters]),
        ).values
        for entry, value in zip(parameters, values, strict=True):
            entry["value"] = Parameter.from_parameter_msg(
                ParameterMessage(name=entry["name"], value=value)
            ).value
        return {"name": node, "parameters": parameters}

    def inspect_all(self):
        nodes = sorted(
            {
                f"{namespace.rstrip('/')}/{name}"
                for name, namespace in self.get_node_names_and_namespaces()
                if not name.startswith("_") and name != self.get_name()
            }
        )
        result = []
        for node in nodes:
            try:
                entry = self.inspect_node(node)
            except (TimeoutError, RuntimeError):
                continue
            if entry is not None:
                result.append(entry)
        return {"nodes": result}

    def update(self, node, changes):
        discovered = {
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        }
        if node not in discovered:
            raise ValueError("Unknown node")
        settings = self.inspect_node(node)
        if settings is None:
            raise ValueError("Live tuning is disabled on this node")
        allowed = {entry["name"]: entry for entry in settings["parameters"]}
        if not isinstance(changes, dict) or not 1 <= len(changes) <= 64:
            raise ValueError("Provide between 1 and 64 numeric changes")
        parameters = []
        for name, value in changes.items():
            entry = allowed.get(name)
            if entry is None or entry["read_only"]:
                raise ValueError(f"Parameter is not tunable: {name}")
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Expected a finite number: {name}")
            if entry["type"] == ParameterType.PARAMETER_INTEGER:
                if value != int(value):
                    raise ValueError(f"Expected an integer: {name}")
                value = int(value)
            else:
                value = float(value)
            parameters.append(Parameter(name, value=value).to_parameter_msg())
        response = self.request(
            node,
            SetParametersAtomically,
            "set_parameters_atomically",
            SetParametersAtomically.Request(parameters=parameters),
        )
        if not response.result.successful:
            raise ValueError(response.result.reason)
        return {"successful": True}


def make_handler(browser, html, token, gamepad_script=""):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *args):
            # Parameter changes are logged by the node that owns the number.
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
                self.respond(200, browser.operator.snapshot())
            elif self.path == "/api/values":
                self.respond(200, browser.inspect_all())
            else:
                self.respond(404, {"error": "Not found"})

        def do_POST(self):
            if self.path not in ("/api/values", "/api/gamepad"):
                self.respond(404, {"error": "Not found"})
                return
            if not secrets.compare_digest(
                self.headers.get("X-Tuning-Token", ""), token
            ):
                self.respond(403, {"error": "Missing developer-menu token"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("Expected a JSON object")
                if self.path == "/api/gamepad":
                    result = browser.operator.update(data)
                else:
                    if not isinstance(data.get("node"), str):
                        raise ValueError(
                            "Provide a node and numeric parameters"
                        )
                    result = browser.update(
                        data["node"], data.get("parameters")
                    )
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                self.respond(400, {"error": str(error)})
                return
            except (TimeoutError, RuntimeError) as error:
                self.respond(503, {"error": str(error)})
                return
            self.respond(200, result)

    return Handler


def main():
    rclpy.init()
    node = ParameterBrowser()
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
        html = (share / "web/tuning.html").read_text()
        handler = make_handler(
            node,
            html,
            secrets.token_urlsafe(32),
            (share / "web/gamepad.js").read_text(),
        )
        with ThreadingHTTPServer(("127.0.0.1", node.port), handler) as server:
            node.get_logger().info(
                f"Developer menu: http://127.0.0.1:{node.port}"
            )
            server.timeout = 0.25
            while rclpy.ok():
                server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        worker.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
