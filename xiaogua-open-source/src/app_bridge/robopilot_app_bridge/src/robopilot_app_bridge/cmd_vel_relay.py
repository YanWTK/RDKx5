"""ROS2 node that subscribes to /cmd_vel and relays it over WebSocket.

This bypasses rosbridge's subscription mechanism which can be unreliable
for bidirectional topics like /cmd_vel.

The node runs a simple WebSocket server on port 19091. ROS1 bridge connects
as a client and receives /cmd_vel messages as JSON.
"""

from __future__ import annotations

import json
import logging
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from websocket_server import WebsocketServer


class CmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        self.declare_parameter("ws_port", 19091)
        self.ws_port = int(self.get_parameter("ws_port").value)

        self._ws_clients: list = []
        self._ws_clients_lock = threading.Lock()
        self._server = WebsocketServer(host="0.0.0.0", port=self.ws_port, loglevel=logging.ERROR)
        self._server.set_fn_new_client(self._on_client_connect)
        self._server.set_fn_client_left(self._on_client_disconnect)
        self._server.set_fn_message_received(self._on_message)

        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self._server_thread = threading.Thread(target=self._server.run_forever, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"cmd_vel relay WebSocket on ws://0.0.0.0:{self.ws_port}")

    def _on_client_connect(self, client, server) -> None:
        old_clients = []
        with self._ws_clients_lock:
            old_clients = [old_client for old_client in self._ws_clients if old_client != client]
            self._ws_clients = [client]

        for old_client in old_clients:
            try:
                self._server._terminate_client_handler(old_client["handler"])
            except Exception:
                pass

        self.get_logger().debug(f"cmd_vel relay client connected: {client['address']}")

    def _on_client_disconnect(self, client, server) -> None:
        with self._ws_clients_lock:
            if client in self._ws_clients:
                self._ws_clients.remove(client)
        self.get_logger().debug(f"cmd_vel relay client disconnected: {client['address']}")

    def _on_message(self, client, server, message) -> None:
        pass  # No incoming messages expected

    def _on_cmd_vel(self, msg: Twist) -> None:
        payload = json.dumps({
            "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
            "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
        })
        self.get_logger().debug(
            f"[ROS2 CMD_VEL RECEIVED] linear.x={msg.linear.x:.3f} angular.z={msg.angular.z:.3f}"
        )
        with self._ws_clients_lock:
            for client in list(self._ws_clients):
                try:
                    self._server.send_message(client, payload)
                except Exception:
                    pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelayNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
