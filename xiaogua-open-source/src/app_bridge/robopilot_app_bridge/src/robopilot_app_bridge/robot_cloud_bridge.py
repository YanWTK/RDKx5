"""MQTT cloud bridge for RoboPilot robot.

Bridges between Alibaba Cloud MQTT and local ROS2 topics/services.
Robot actively connects to cloud server — never exposes local ports to public internet.
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def _load_env(env_path: str | None = None) -> dict:
    """Load configuration from .env file. Never log the password."""
    if env_path is None:
        env_path = os.path.join(
            os.path.dirname(__file__), "..", "..", "config", "mqtt_bridge.env"
        )
    env_path = str(Path(env_path).resolve())
    if not Path(env_path).exists():
        print(f"FATAL: config file not found: {env_path}", file=sys.stderr)
        sys.exit(1)

    cfg = {}
    try:
        from dotenv import load_dotenv
        load_dotenv(env_path, override=True)
    except ImportError:
        # Fallback: manual parse
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k.strip(), v.strip())

    required = ["MQTT_HOST", "MQTT_PASS", "MQTT_USER"]
    for key in required:
        val = os.environ.get(key, "")
        if not val or "请填写" in val:
            print(f"FATAL: {key} is not configured in {env_path}", file=sys.stderr)
            sys.exit(1)

    cfg["host"] = os.environ["MQTT_HOST"]
    cfg["port"] = int(os.environ.get("MQTT_PORT", "1883"))
    cfg["user"] = os.environ["MQTT_USER"]
    cfg["password"] = os.environ["MQTT_PASS"]
    cfg["robot_id"] = os.environ.get("ROBOT_ID", "001")
    cfg["keepalive"] = int(os.environ.get("MQTT_KEEPALIVE", "30"))
    cfg["client_id"] = os.environ.get("MQTT_CLIENT_ID", f"robot_{cfg['robot_id']}_bridge")
    cfg["enable_goal"] = os.environ.get("MQTT_ENABLE_GOAL", "false").lower() == "true"
    cfg["publish_scan_summary"] = os.environ.get("MQTT_PUBLISH_SCAN_SUMMARY", "true").lower() == "true"
    cfg["heartbeat_interval"] = int(os.environ.get("MQTT_HEARTBEAT_INTERVAL", "5"))
    return cfg


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ts() -> int:
    return int(time.time())


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Velocity limits
LIN_X_MAX = 0.5
ANG_Z_MAX = 1.2


def _clamp_vel(linear_x: float, angular_z: float) -> tuple[float, float]:
    return (
        max(-LIN_X_MAX, min(LIN_X_MAX, linear_x)),
        max(-ANG_Z_MAX, min(ANG_Z_MAX, angular_z)),
    )


# Emergency stop keywords
_ESTOP_KEYWORDS = {"急停", "紧急停止", "马上停", "危险", "emergency_stop", "emergency"}


# ---------------------------------------------------------------------------
# Main bridge node
# ---------------------------------------------------------------------------

class RobotCloudBridge(Node):
    def __init__(self, cfg: dict) -> None:
        super().__init__("robot_cloud_bridge")
        self.cfg = cfg
        self.robot_id = cfg["robot_id"]
        self._mqtt: mqtt.Client | None = None
        self._mqtt_connected = threading.Event()
        self._emergency_stop_until = 0.0  # monotonic time
        self._shutdown = False

        # ---- ROS publishers ----
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._voice_cmd_pub = self.create_publisher(String, "/voice_cmd", 10)

        self._goal_pub = None
        if cfg["enable_goal"]:
            self._goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            self.get_logger().info("Goal publishing ENABLED on /move_base_simple/goal")
        else:
            self.get_logger().info("Goal publishing DISABLED (set MQTT_ENABLE_GOAL=true to enable)")

        # ---- ROS subscribers (with rate limiting) ----
        self._last_pub: dict[str, float] = {}
        self._last_mode_status: str = ""

        self.create_subscription(String, "/robot_status", self._on_robot_status, 10)
        self.create_subscription(String, "/mode/status", self._on_mode_status, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self._on_robot_pose, 10)

        # Try subscribing to /odom (nav_msgs.Odometry) — may not exist
        try:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self._odom_available = True
        except Exception:
            self._odom_available = False
            self.get_logger().warn("Could not import nav_msgs.Odometry — /odom subscription disabled")

        # Voltage — try Float32, fallback Float64
        try:
            self.create_subscription(Float32, "/voltage", self._on_voltage, 10)
            self._voltage_type = "float32"
        except Exception:
            try:
                from std_msgs.msg import Float64
                self.create_subscription(Float64, "/voltage", self._on_voltage, 10)
                self._voltage_type = "float64"
            except Exception:
                self._voltage_type = "none"
                self.get_logger().warn("Could not subscribe to /voltage")

        # Scan summary
        if cfg["publish_scan_summary"]:
            try:
                from sensor_msgs.msg import LaserScan
                self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
                self._scan_available = True
            except Exception:
                self._scan_available = False
                self.get_logger().warn("Could not import sensor_msgs.LaserScan")
        else:
            self._scan_available = False

        # Diagnostics (optional, low rate)
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
            self._diag_available = True
        except Exception:
            self._diag_available = False

        # ---- ROS service clients ----
        self._service_clients: dict[str, rclpy.client.Client] = {}
        service_names = [
            "/mode/switch_to_mapping",
            "/mode/switch_to_navigation",
            "/mode/switch_to_patrol",
            "/mode/get_status",
            "/mapping/start",
            "/mapping/save",
            "/mapping/stop",
        ]
        for svc_name in service_names:
            try:
                client = self.create_client(Trigger, svc_name)
                self._service_clients[svc_name] = client
            except Exception as e:
                self.get_logger().warn(f"Could not create client for {svc_name}: {e}")

        # ---- Heartbeat timer ----
        self.create_timer(float(cfg["heartbeat_interval"]), self._heartbeat_tick)

        self.get_logger().info("RobotCloudBridge node initialized")

    # -------------------------------------------------------------------
    # MQTT setup
    # -------------------------------------------------------------------

    def start_mqtt(self) -> None:
        """Connect to MQTT broker and start network loop in background thread."""
        cfg = self.cfg
        # paho-mqtt 2.x requires CallbackAPIVersion; use VERSION1 for backward-compatible callbacks
        client_kwargs = {"client_id": cfg["client_id"], "protocol": mqtt.MQTTv311}
        if hasattr(mqtt, "CallbackAPIVersion"):
            client_kwargs["callback_api_version"] = mqtt.CallbackAPIVersion.VERSION1
        self._mqtt = mqtt.Client(**client_kwargs)
        self._mqtt.username_pw_set(cfg["user"], cfg["password"])
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=30)
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message = self._on_mqtt_message

        self.get_logger().info(f"Connecting to MQTT {cfg['host']}:{cfg['port']} as {cfg['user']}...")
        try:
            self._mqtt.connect(cfg["host"], cfg["port"], cfg["keepalive"])
        except Exception as e:
            self.get_logger().error(f"MQTT initial connect failed: {e} — will retry in background")
        self._mqtt.loop_start()

    def stop_mqtt(self) -> None:
        if self._mqtt:
            self._mqtt.loop_stop()
            try:
                self._mqtt.disconnect()
            except Exception:
                pass

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            self._mqtt_connected.set()
            self._subscribe_topics()
        else:
            self.get_logger().error(f"MQTT connect returned rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        self._mqtt_connected.clear()
        if rc != 0:
            self.get_logger().warn(f"MQTT disconnected unexpectedly (rc={rc}), will auto-reconnect")

    def _subscribe_topics(self):
        rid = self.robot_id
        topics = [
            f"robot/{rid}/cmd/vel",
            f"robot/{rid}/cmd/voice",
            f"robot/{rid}/cmd/stop",
            f"robot/{rid}/cmd/emergency_stop",
            f"robot/{rid}/service/mode",
            f"robot/{rid}/service/mapping",
        ]
        if self.cfg["enable_goal"]:
            topics.append(f"robot/{rid}/cmd/goal")
        for t in topics:
            self._mqtt.subscribe(t, qos=1)
            self.get_logger().info(f"Subscribed MQTT: {t}")

    # -------------------------------------------------------------------
    # MQTT message dispatch
    # -------------------------------------------------------------------

    def _on_mqtt_message(self, client, userdata, msg: mqtt.MQTTMessage):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            self._publish_result(topic, success=False, message=f"JSON parse error: {e}")
            return

        rid = self.robot_id
        try:
            if topic == f"robot/{rid}/cmd/vel":
                self._handle_cmd_vel(payload)
            elif topic == f"robot/{rid}/cmd/voice":
                self._handle_cmd_voice(payload)
            elif topic == f"robot/{rid}/cmd/stop":
                self._handle_cmd_stop(payload)
            elif topic == f"robot/{rid}/cmd/emergency_stop":
                self._handle_emergency_stop(payload)
            elif topic == f"robot/{rid}/service/mode":
                self._handle_service_mode(payload)
            elif topic == f"robot/{rid}/service/mapping":
                self._handle_service_mapping(payload)
            elif topic == f"robot/{rid}/cmd/goal":
                self._handle_cmd_goal(payload)
            else:
                self.get_logger().warn(f"Unknown MQTT topic: {topic}")
        except Exception as e:
            self._publish_result(topic, success=False, message=f"Handler error: {e}")

    # -------------------------------------------------------------------
    # Command handlers
    # -------------------------------------------------------------------

    def _handle_cmd_vel(self, payload: dict):
        # Emergency stop guard
        if time.monotonic() < self._emergency_stop_until:
            self._publish_result("cmd/vel", success=False, message="blocked by emergency stop")
            return

        linear = payload.get("linear", {})
        angular = payload.get("angular", {})
        lx = float(linear.get("x", 0.0))
        az = float(angular.get("z", 0.0))
        lx, az = _clamp_vel(lx, az)

        twist = Twist()
        twist.linear.x = lx
        twist.angular.z = az
        self._cmd_vel_pub.publish(twist)
        self._publish_result("cmd/vel", success=True,
                             message=f"vel published: linear.x={lx:.3f}, angular.z={az:.3f}")

    def _handle_cmd_voice(self, payload: dict):
        data = str(payload.get("data", ""))
        msg = String()
        msg.data = data
        self._voice_cmd_pub.publish(msg)
        self._publish_result("cmd/voice", success=True, message=f"voice published: {data}")

        # Check for emergency stop keywords
        if data in _ESTOP_KEYWORDS:
            self.get_logger().warn(f"Emergency stop triggered by voice: {data}")
            self._do_emergency_stop("voice_keyword")

    def _handle_cmd_stop(self, payload: dict):
        self.get_logger().info("Stop command received")
        twist = Twist()  # all zeros
        for _ in range(3):
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.08)
        self._publish_result("cmd/stop", success=True, message="stop published (3x zero)")

    def _handle_emergency_stop(self, payload: dict):
        reason = payload.get("reason", "unknown")
        self._do_emergency_stop(reason)

    def _do_emergency_stop(self, reason: str):
        self.get_logger().warn(f"EMERGENCY STOP: reason={reason}")
        self._emergency_stop_until = time.monotonic() + 2.0
        twist = Twist()  # all zeros
        for _ in range(5):
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._publish_result("emergency_stop", success=True,
                             message=f"emergency stop executed, blocking 2s, reason={reason}")

    def _handle_service_mode(self, payload: dict):
        target = payload.get("target", "")
        msg_id = payload.get("msg_id", "")
        svc_map = {
            "mapping": "/mode/switch_to_mapping",
            "navigation": "/mode/switch_to_navigation",
            "patrol": "/mode/switch_to_patrol",
            "status": "/mode/get_status",
        }
        if target not in svc_map:
            self._publish_result("service/mode", success=False, msg_id=msg_id,
                                 message=f"unknown target: {target}")
            return
        self._call_service_async(svc_map[target], "mode", msg_id)

    def _handle_service_mapping(self, payload: dict):
        action = payload.get("action", "")
        msg_id = payload.get("msg_id", "")
        svc_map = {
            "start": "/mapping/start",
            "save": "/mapping/save",
            "stop": "/mapping/stop",
        }
        if action not in svc_map:
            self._publish_result("service/mapping", success=False, msg_id=msg_id,
                                 message=f"unknown action: {action}")
            return
        self._call_service_async(svc_map[action], "mapping", msg_id)

    def _handle_cmd_goal(self, payload: dict):
        if not self.cfg["enable_goal"]:
            self._publish_result("cmd/goal", success=False, message="goal command received but not enabled")
            return

        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        theta = float(payload.get("theta", 0.0))
        frame_id = payload.get("frame_id", "map")

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        half = theta * 0.5
        goal.pose.orientation.z = math.sin(half)
        goal.pose.orientation.w = math.cos(half)
        self._goal_pub.publish(goal)
        self._publish_result("cmd/goal", success=True,
                             message=f"goal published: ({x}, {y}, theta={theta})")

    # -------------------------------------------------------------------
    # ROS service calls (non-blocking)
    # -------------------------------------------------------------------

    def _call_service_async(self, service_name: str, command: str, msg_id: str):
        """Call a ROS2 Trigger service in a background thread."""
        client = self._service_clients.get(service_name)
        if client is None:
            self._publish_result(f"service/{command}", success=False, msg_id=msg_id,
                                 message=f"service client not available: {service_name}")
            return

        def _call():
            try:
                if not client.wait_for_service(timeout_sec=5.0):
                    self._publish_result(f"service/{command}", success=False, msg_id=msg_id,
                                         service=service_name,
                                         message=f"service not available: {service_name}")
                    return
                req = Trigger.Request()
                future = client.call_async(req)
                # Wait with timeout
                deadline = time.monotonic() + 10.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if future.done():
                    resp = future.result()
                    self._publish_result(f"service/{command}", success=resp.success, msg_id=msg_id,
                                         service=service_name,
                                         message=resp.message)
                else:
                    self._publish_result(f"service/{command}", success=False, msg_id=msg_id,
                                         service=service_name,
                                         message=f"service call timed out: {service_name}")
            except Exception as e:
                self._publish_result(f"service/{command}", success=False, msg_id=msg_id,
                                     service=service_name,
                                     message=f"service call error: {e}")

        threading.Thread(target=_call, daemon=True).start()

    # -------------------------------------------------------------------
    # ROS subscribers → MQTT
    # -------------------------------------------------------------------

    def _should_pub(self, key: str, interval: float) -> bool:
        now = time.monotonic()
        last = self._last_pub.get(key, 0.0)
        if now - last >= interval:
            self._last_pub[key] = now
            return True
        return False

    def _publish_mqtt(self, sub_topic: str, payload: dict):
        if not self._mqtt_connected.is_set():
            return
        full_topic = f"robot/{self.robot_id}/{sub_topic}"
        payload["robot_id"] = self.robot_id
        payload["ts"] = _ts()
        try:
            self._mqtt.publish(full_topic, json.dumps(payload, ensure_ascii=False), qos=0)
        except Exception as e:
            self.get_logger().warn(f"MQTT publish error on {full_topic}: {e}")

    def _on_robot_status(self, msg: String):
        if not self._should_pub("status", 1.0):
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        self._publish_mqtt("status", {"data": data, "source": "ros"})

    def _on_mode_status(self, msg: String):
        data_str = msg.data
        # Only publish on change or at 1 Hz
        if data_str == self._last_mode_status and not self._should_pub("mode_status", 1.0):
            return
        self._last_mode_status = data_str
        try:
            data = json.loads(data_str)
        except Exception:
            data = {"raw": data_str}
        self._publish_mqtt("mode_status", {"data": data, "source": "ros"})

    def _on_voltage(self, msg):
        if not self._should_pub("battery", 1.0):
            return
        voltage = float(msg.data)
        # Rough percent estimate for 3S LiPo (12.6V full, 9.6V empty)
        percent = max(0, min(100, int((voltage - 9.6) / (12.6 - 9.6) * 100)))
        self._publish_mqtt("battery", {"voltage": round(voltage, 2), "percent": percent, "source": "ros"})

    def _on_robot_pose(self, msg: PoseStamped):
        if not self._should_pub("pose", 0.5):  # 2 Hz
            return
        yaw = _yaw_from_quaternion(msg.pose.orientation)
        self._publish_mqtt("pose", {
            "x": round(msg.pose.position.x, 3),
            "y": round(msg.pose.position.y, 3),
            "yaw": round(yaw, 3),
            "frame_id": msg.header.frame_id,
            "source_topic": "/robot_pose",
            "source": "ros",
        })

    def _on_odom(self, msg):
        if not self._should_pub("pose", 0.5):  # 2 Hz, same slot as robot_pose
            return
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._publish_mqtt("pose", {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "yaw": round(yaw, 3),
            "frame_id": msg.header.frame_id,
            "source_topic": "/odom",
            "source": "ros",
        })

    def _on_scan(self, msg):
        if not self._should_pub("scan", 1.0):
            return
        ranges = msg.ranges
        if not ranges:
            return
        # Filter out inf/nan
        valid = [r for r in ranges if msg.range_min <= r <= msg.range_max]
        min_range = min(valid) if valid else float("inf")
        # Obstacle in front: ±15 degrees around 0
        n = len(ranges)
        front_indices = list(range(max(0, n // 2 - 15), min(n, n // 2 + 15)))
        front_ranges = [ranges[i] for i in front_indices if msg.range_min <= ranges[i] <= msg.range_max]
        obstacle_front = any(r < 0.5 for r in front_ranges) if front_ranges else False
        self._publish_mqtt("scan", {
            "minRange": round(min_range, 2),
            "obstacleFront": obstacle_front,
            "source": "ros",
        })

    def _on_diagnostics(self, msg):
        if not self._should_pub("diagnostics", 2.0):  # 0.5 Hz
            return
        items = []
        for status in msg.status[:5]:  # limit items
            items.append({
                "name": status.name,
                "level": status.level,
                "message": status.message,
            })
        self._publish_mqtt("diagnostics", {"items": items, "source": "ros"})

    # -------------------------------------------------------------------
    # Heartbeat
    # -------------------------------------------------------------------

    def _heartbeat_tick(self):
        self._publish_mqtt("heartbeat", {
            "online": True,
            "ros_connected": True,
            "mqtt_connected": self._mqtt_connected.is_set(),
            "node": "robot_cloud_bridge",
        })

    # -------------------------------------------------------------------
    # Result publishing
    # -------------------------------------------------------------------

    def _publish_result(self, command: str, success: bool, message: str,
                        msg_id: str = "", service: str = ""):
        payload = {
            "success": success,
            "command": command,
            "message": message,
            "ts": _ts(),
            "source": "robot_cloud_bridge",
        }
        if msg_id:
            payload["msg_id"] = msg_id
        if service:
            payload["service"] = service
        self._publish_mqtt("result", payload)

    # -------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------

    def shutdown(self):
        self._shutdown = True
        # Send a final zero velocity on shutdown
        try:
            twist = Twist()
            self._cmd_vel_pub.publish(twist)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    cfg = _load_env()

    rclpy.init(args=args)
    node = RobotCloudBridge(cfg)

    # Handle signals gracefully
    def _signal_handler(sig, frame):
        node.get_logger().info(f"Received signal {sig}, shutting down...")
        node.shutdown()
        node.stop_mqtt()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Start MQTT connection
    node.start_mqtt()
    node.get_logger().info("Robot cloud bridge running. Press Ctrl+C to stop.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.stop_mqtt()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
