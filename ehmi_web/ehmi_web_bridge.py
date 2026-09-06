#!/usr/bin/env python3
"""
Minimal TurtleBot3 eHMI bridge
ROS 2 Humble / Python standard library only.

ROS inputs:
  /ehmi_state      std_msgs/msg/String
  /ehmi_countdown  std_msgs/msg/Int32

HTTP:
  http://<robot-ip>:8080/
  GET /api/state

No Flask / WebSocket / rosbridge required.
"""

import json
import os
from pathlib import Path
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


PORT = 8080
HTML_PATH = os.path.expanduser(os.environ.get(
    "TB3_EHMI_HTML",
    str(Path(__file__).resolve().with_name("turtlebot3_ehmi_v4_countdown.html")),
))


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = "APPROACH"
        self.countdown = 0

    def set_state(self, value):
        with self.lock:
            self.state = value

    def set_countdown(self, value):
        with self.lock:
            self.countdown = int(value)

    def snapshot(self):
        with self.lock:
            return {
                "state": self.state,
                "countdown": self.countdown,
            }


shared = SharedState()


class EHMINode(Node):
    def __init__(self):
        super().__init__("ehmi_web_bridge")

        self.create_subscription(
            String,
            "/ehmi_state",
            self.on_state,
            10,
        )

        self.create_subscription(
            Int32,
            "/ehmi_countdown",
            self.on_countdown,
            10,
        )

        self.get_logger().info("eHMI bridge ready")
        self.get_logger().info("Subscribe: /ehmi_state [std_msgs/String]")
        self.get_logger().info("Subscribe: /ehmi_countdown [std_msgs/Int32]")

    def on_state(self, msg):
        shared.set_state(msg.data.strip())
        self.get_logger().info(f"eHMI state -> {msg.data.strip()}")

    def on_countdown(self, msg):
        shared.set_countdown(msg.data)


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/state":
            body = json.dumps(shared.snapshot()).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return

        if self.path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    body = f.read()
                self._send(200, "text/html; charset=utf-8", body)
            except FileNotFoundError:
                msg = (
                    f"HTML not found:\n{HTML_PATH}\n\n"
                    "Restore the bundled v4 HTML or set TB3_EHMI_HTML to an existing file."
                )
                self._send(404, "text/plain; charset=utf-8", msg.encode("utf-8"))
            return

        self._send(404, "text/plain; charset=utf-8", b"Not found")

    def log_message(self, fmt, *args):
        # Keep terminal clean.
        return


def run_http():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"[eHMI] HTTP server: 0.0.0.0:{PORT}")
    server.serve_forever()


def main():
    rclpy.init()

    http_thread = threading.Thread(target=run_http, daemon=True)
    http_thread.start()

    node = EHMINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
