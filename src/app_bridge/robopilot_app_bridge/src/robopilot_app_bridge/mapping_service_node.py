from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .common import build_static_map, save_occupancy_grid


class MappingServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("robopilot_mapping_service")

        self.declare_parameter("save_dir", "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map")
        self.declare_parameter("save_basename", "cartographer_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_width_cells", 240)
        self.declare_parameter("map_height_cells", 240)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("control_topic", "/robopilot/mapping/control")

        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.save_basename = str(self.get_parameter("save_basename").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.map_width_cells = int(self.get_parameter("map_width_cells").value)
        self.map_height_cells = int(self.get_parameter("map_height_cells").value)
        self.map_resolution = float(self.get_parameter("map_resolution").value)
        self.control_topic = str(self.get_parameter("control_topic").value)

        self.origin_x = -0.5 * self.map_width_cells * self.map_resolution
        self.origin_y = -0.5 * self.map_height_cells * self.map_resolution

        self._state_lock = threading.Lock()
        self._mapping_active = False
        self._latest_saved_map = ""
        self._latest_grid: np.ndarray | None = None
        self._base_map = build_static_map(self.map_width_cells, self.map_height_cells)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        topic_qos = 10

        self._control_pub = self.create_publisher(String, self.control_topic, topic_qos)
        self.create_subscription(
            OccupancyGrid,
            "/cartographer_map",
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            map_qos,
        )
        self.create_service(
            Trigger,
            "/mapping/start",
            self._on_mapping_start,
        )
        self.create_service(
            Trigger,
            "/mapping/save",
            self._on_mapping_save,
        )
        self.create_service(
            Trigger,
            "/mapping/stop",
            self._on_mapping_stop,
        )
        self.create_service(
            Trigger,
            "/mode/switch_to_mapping",
            self._on_mode_switch_to_mapping,
        )
        self.create_service(
            Trigger,
            "/mode/switch_to_navigation",
            self._on_mode_switch_to_navigation,
        )
        self.create_service(
            Trigger,
            "/mode/switch_to_patrol",
            self._on_mode_switch_to_patrol,
        )
        self.create_service(
            Trigger,
            "/mode/get_status",
            self._on_mode_get_status,
        )

        self.get_logger().info(
            f"mapping services ready on /mapping/start|save|stop, /mode/switch_to_*, control topic {self.control_topic}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            width = int(msg.info.width)
            height = int(msg.info.height)
            grid = np.array(msg.data, dtype=np.int8).reshape(height, width)
        except Exception:
            return
        with self._state_lock:
            self._latest_grid = grid

    def _latest_grid_for_save(self) -> np.ndarray:
        with self._state_lock:
            if self._latest_grid is not None:
                return self._latest_grid.copy()
        return self._base_map.copy()

    def _publish_control(self, command: str) -> None:
        self.get_logger().info(f"publishing mapping control command: {command}")
        self._control_pub.publish(String(data=command))

    def _on_mapping_start(self, request, response):
        self.get_logger().info("service /mapping/start requested")
        with self._state_lock:
            self._mapping_active = True
        self._publish_control("switch_to_mapping")
        response.success = True
        response.message = "mapping started"
        return response

    def _on_mapping_stop(self, request, response):
        self.get_logger().info("service /mapping/stop requested")
        with self._state_lock:
            self._mapping_active = False
        self._publish_control("switch_to_navigation")
        response.success = True
        response.message = "mapping stopped"
        return response

    def _on_mapping_save(self, request, response):
        self.get_logger().info("service /mapping/save requested")
        saved = save_occupancy_grid(
            self._latest_grid_for_save(),
            self.save_dir,
            self.save_basename,
            self.map_resolution,
            self.origin_x,
            self.origin_y,
        )
        with self._state_lock:
            self._latest_saved_map = str(saved)
        self._publish_control("switch_to_navigation")
        response.success = True
        response.message = f"saved map to {self._latest_saved_map}.yaml"
        return response

    def _on_mode_switch_to_mapping(self, request, response):
        self.get_logger().info("service /mode/switch_to_mapping requested")
        self._publish_control("switch_to_mapping")
        response.success = True
        response.message = "mapping switch command sent"
        return response

    def _on_mode_switch_to_navigation(self, request, response):
        self.get_logger().info("service /mode/switch_to_navigation requested")
        self._publish_control("switch_to_navigation")
        response.success = True
        response.message = "navigation switch command sent"
        return response

    def _on_mode_switch_to_patrol(self, request, response):
        self.get_logger().info("service /mode/switch_to_patrol requested")
        self._publish_control("switch_to_patrol")
        response.success = True
        response.message = "switching to patrol"
        return response

    def _on_mode_get_status(self, request, response):
        self.get_logger().info("service /mode/get_status requested")
        self._publish_control("get_status")
        response.success = True
        response.message = "status requested"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingServiceNode()
    try:
        rclpy.spin(node)
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
