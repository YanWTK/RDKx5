#!/usr/bin/env python3
"""ROS 2 node: reSpeaker XVF3800 DOA -> TF + RVIZ Marker.

阶段一目标
-----------
* 订阅 ``/xvf3800/doa_deg`` (Float32, 度) 和 ``/xvf3800/vad`` (Bool)
* 启动时一次性发布 static TF: ``base_link -> microphone_link`` (按 yaml 里位姿)
* 每次收到 DOA 时发布 dynamic TF: ``microphone_link -> sound_source`` (仅绕 Z 旋转)
* 同步发布 ``visualization_msgs/Marker`` (ARROW) 指向说话方向

所有可调参数 (麦克风位姿 / DOA offset / 射线长度 / 颜色 / 行为开关) 都从 yaml 读，
阶段一校准时只改 yaml 不动代码。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """绕 Z 轴的 yaw 转 quaternion (ROS 标准 X 前 Y 左 Z 上右手系)."""
    half = yaw_rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class DoaVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('doa_visualizer_node')

        # ====== 声明并读取所有参数 ======
        self.declare_parameter('doa_topic', '/xvf3800/doa_deg')
        self.declare_parameter('vad_topic', '/xvf3800/vad')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('mic_frame', 'microphone_link')
        self.declare_parameter('sound_frame', 'sound_source')
        self.declare_parameter('mic_x', 0.10)
        self.declare_parameter('mic_y', 0.0)
        self.declare_parameter('mic_z', 0.30)
        self.declare_parameter('mic_yaw', 0.0)
        self.declare_parameter('doa_offset_deg', 0.0)
        self.declare_parameter('doa_ccw', 1.0)
        self.declare_parameter('arrow_length', 1.5)
        self.declare_parameter('marker_lifetime_sec', 1.5)
        self.declare_parameter('marker_color_r', 0.1)
        self.declare_parameter('marker_color_g', 0.9)
        self.declare_parameter('marker_color_b', 0.3)
        self.declare_parameter('marker_color_a', 0.9)
        self.declare_parameter('marker_shaft_diameter', 0.025)
        self.declare_parameter('marker_head_diameter', 0.06)
        self.declare_parameter('marker_head_length', 0.08)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('keep_publishing_on_vad_loss', False)

        p = self.get_parameter
        self.doa_topic = p('doa_topic').value
        self.vad_topic = p('vad_topic').value
        self.parent_frame = p('parent_frame').value
        self.mic_frame = p('mic_frame').value
        self.sound_frame = p('sound_frame').value
        self.mic_x = float(p('mic_x').value)
        self.mic_y = float(p('mic_y').value)
        self.mic_z = float(p('mic_z').value)
        self.mic_yaw = float(p('mic_yaw').value)
        self.doa_offset_deg = float(p('doa_offset_deg').value)
        self.doa_ccw = float(p('doa_ccw').value)
        self.arrow_length = float(p('arrow_length').value)
        self.marker_lifetime_sec = float(p('marker_lifetime_sec').value)
        self.marker_color = (
            float(p('marker_color_r').value),
            float(p('marker_color_g').value),
            float(p('marker_color_b').value),
            float(p('marker_color_a').value),
        )
        self.marker_shaft_diameter = float(p('marker_shaft_diameter').value)
        self.marker_head_diameter = float(p('marker_head_diameter').value)
        self.marker_head_length = float(p('marker_head_length').value)
        self.publish_tf = bool(p('publish_tf').value)
        self.publish_marker = bool(p('publish_marker').value)
        self.keep_publishing_on_vad_loss = bool(p('keep_publishing_on_vad_loss').value)

        # ====== 状态 ======
        self._last_doa_deg: float | None = None
        self._last_vad: bool = False

        # ====== TF broadcast 句柄 ======
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_microphone_link_static_tf()

        # ====== Marker 发布 ======
        self._marker_pub = self.create_publisher(
            Marker, '/doa_visualizer/marker', 10
        ) if self.publish_marker else None

        # ====== 订阅 ======
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float32, self.doa_topic, self._on_doa, qos)
        self.create_subscription(Bool, self.vad_topic, self._on_vad, qos)

        self.get_logger().info(
            f'doa_visualizer 已启动: doa={self.doa_topic} vad={self.vad_topic} '
            f'offset={self.doa_offset_deg}deg ccw={self.doa_ccw} '
            f'mic=({self.mic_x},{self.mic_y},{self.mic_z}) '
            f'arrow={self.arrow_length}m'
        )

    # ------------------------------------------------------------------ TF

    def _publish_microphone_link_static_tf(self) -> None:
        """启动时一次性广播 base_link -> microphone_link (latched)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.mic_frame
        t.transform.translation = Vector3(
            x=self.mic_x, y=self.mic_y, z=self.mic_z
        )
        t.transform.rotation = yaw_to_quaternion(self.mic_yaw)
        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'广播 static TF: {self.parent_frame} -> {self.mic_frame} '
            f'xyz=({self.mic_x:.3f},{self.mic_y:.3f},{self.mic_z:.3f}) '
            f'yaw={math.degrees(self.mic_yaw):.1f}deg'
        )

    # ------------------------------------------------------------- 回调

    def _on_doa(self, msg: Float32) -> None:
        self._last_doa_deg = float(msg.data)
        # 不管 VAD, 都缓存最新 DOA. 实际发不发由 _publish_now 决定.

    def _on_vad(self, msg: Bool) -> None:
        self._last_vad = bool(msg.data)

    # ----------------------------------------------------------- 发布

    def _compute_yaw_rad(self) -> float | None:
        if self._last_doa_deg is None:
            return None
        deg = (self._last_doa_deg + self.doa_offset_deg) * self.doa_ccw
        # 归一化到 [-180, 180]
        deg = ((deg + 180.0) % 360.0) - 180.0
        return math.radians(deg)

    def _publish_now(self) -> None:
        yaw_rad = self._compute_yaw_rad()
        if yaw_rad is None:
            return
        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(yaw_rad)
        # 水平面内箭头端点
        end_x = self.arrow_length * math.cos(yaw_rad)
        end_y = self.arrow_length * math.sin(yaw_rad)

        if self.publish_tf and self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.mic_frame
            t.child_frame_id = self.sound_frame
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

        if self.publish_marker and self._marker_pub is not None:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.mic_frame
            m.ns = 'doa_arrow'
            m.id = 0
            m.type = Marker.ARROW
            m.action = Marker.ADD
            # ARROW 用 points[0]=start, points[1]=end, 方向由 points 表达
            m.points = [Point(x=0.0, y=0.0, z=0.0),
                        Point(x=end_x, y=end_y, z=0.0)]
            m.pose.orientation.w = 1.0  # 整体姿态=identity
            m.scale.x = self.marker_shaft_diameter  # shaft diameter
            m.scale.y = self.marker_head_diameter   # head diameter
            m.scale.z = self.marker_head_length     # head length
            m.color.r, m.color.g, m.color.b, m.color.a = self.marker_color
            lifetime_sec = max(0.05, self.marker_lifetime_sec)
            from builtin_interfaces.msg import Duration
            m.lifetime = Duration(sec=int(lifetime_sec),
                                  nanosec=int((lifetime_sec - int(lifetime_sec)) * 1e9))
            m.frame_locked = False
            self._marker_pub.publish(m)

    # ------------------------------------------------------------ tick

    def tick(self) -> None:
        """主循环: 仅在 ROS 外部 spin 调用, 这里走 main() 内的 spin."""
        # VAD 上升沿: 立刻推一发
        if self._last_vad and self._last_doa_deg is not None:
            self._publish_now()
        elif not self._last_vad and not self.keep_publishing_on_vad_loss:
            # VAD 关闭且不要求继续发 -> 不动作, 让 marker 自然消逝
            return
        else:
            self._publish_now()


def main() -> None:
    rclpy.init()
    node = DoaVisualizer()
    tick_period = 0.1  # 100ms: 不抢占 DOA 实际频率, 兜底刷新
    node.create_timer(tick_period, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()