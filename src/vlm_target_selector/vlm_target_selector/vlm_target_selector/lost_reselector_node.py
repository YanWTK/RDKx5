#!/usr/bin/env python3
"""Re-call the VLM selector when the tracked target is lost too long."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from vlm_target_msgs.srv import SelectTarget


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class LostReselectorNode(Node):
    """Watch tracker status and ask the VLM selector to reselect after loss."""

    def __init__(self):
        super().__init__('vlm_lost_reselector')

        self.declare_parameter('status_topic', '/object_tracker/status')
        self.declare_parameter('result_topic', '/vlm_target_selector/reselector_status')
        self.declare_parameter('service_name', '/vlm_target_selector/select_target')
        self.declare_parameter('target_name', '我的白色水杯')
        self.declare_parameter('target_name_topic', '/vlm_target_selector/current_target_name')
        self.declare_parameter('lost_reselect_delay_sec', 1.0)
        self.declare_parameter('max_lost_no_object_sec', 8.0)
        self.declare_parameter('reselect_cooldown_sec', 5.0)
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('trigger_on_lost_final', True)
        self.declare_parameter('wait_service_timeout_sec', 0.2)

        self.status_topic = self.get_parameter('status_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.service_name = self.get_parameter('service_name').value
        self.target_name = str(self.get_parameter('target_name').value).strip()
        self.target_name_topic = str(self.get_parameter('target_name_topic').value).strip()
        self.lost_delay = float(self.get_parameter('lost_reselect_delay_sec').value)
        self.max_lost_no_object = float(
            self.get_parameter('max_lost_no_object_sec').value
        )
        self.cooldown = float(self.get_parameter('reselect_cooldown_sec').value)
        self.save_debug_images = _as_bool(self.get_parameter('save_debug_images').value)
        self.trigger_on_lost_final = _as_bool(
            self.get_parameter('trigger_on_lost_final').value
        )
        self.wait_service_timeout = float(
            self.get_parameter('wait_service_timeout_sec').value
        )

        self._client = self.create_client(SelectTarget, self.service_name)
        self._result_pub = self.create_publisher(String, self.result_topic, 10)
        self.create_subscription(String, self.status_topic, self._on_status, 10)
        if self.target_name_topic:
            target_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                self.target_name_topic,
                self._on_target_name,
                target_qos,
            )
        self.create_timer(0.2, self._on_timer)

        self._lost_since = None
        self._last_reselect_time = 0.0
        self._request_in_flight = False
        self._last_status = {}
        self._final_no_object = False

        self.get_logger().info(
            'vlm_lost_reselector started | '
            f'status={self.status_topic} | service={self.service_name} | '
            f'target={self.target_name or "(waiting)"} | '
            f'target_topic={self.target_name_topic or "(disabled)"} | '
            f'lost_delay={self.lost_delay:.2f}s | '
            f'no_object_after={self.max_lost_no_object:.2f}s | '
            f'cooldown={self.cooldown:.2f}s'
        )

    def _on_target_name(self, msg: String) -> None:
        target = msg.data.strip()
        if not target:
            return
        if target != self.target_name:
            self.get_logger().info(f'current VLM target updated: {target}')
        self.target_name = target
        self._lost_since = None
        self._final_no_object = False

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'failed to parse tracker status JSON: {exc}')
            return

        now = time.monotonic()
        selected_active = bool(status.get('selected_active', False))
        selected_track_id = int(status.get('selected_track_id') or 0)
        lost_final = bool(status.get('selection_lost_final', False))

        self._last_status = status

        if selected_active:
            self._lost_since = None
            if self._final_no_object:
                self._final_no_object = False
                self._publish_result(
                    state='tracking',
                    object_present=True,
                    final_no_object=False,
                    reason='selected target active again',
                )
            return

        lost_selected_track = selected_track_id > 0 and not selected_active
        final_lost_event = self.trigger_on_lost_final and lost_final
        if lost_selected_track or final_lost_event:
            if self._lost_since is None:
                self._lost_since = now
            if final_lost_event:
                self._maybe_call_vlm('selection_lost_final')
            return

        self._lost_since = None

    def _on_timer(self) -> None:
        if self._lost_since is None:
            return

        lost_for = time.monotonic() - self._lost_since
        if (
            self.max_lost_no_object > 0.0
            and lost_for >= self.max_lost_no_object
            and not self._final_no_object
        ):
            self._declare_no_object(lost_for)
            return

        if lost_for < self.lost_delay:
            return

        self._maybe_call_vlm(f'lost_for={lost_for:.2f}s')

    def _maybe_call_vlm(self, reason: str) -> None:
        if self._final_no_object:
            return

        now = time.monotonic()
        if self._request_in_flight:
            return
        if not self.target_name:
            self.get_logger().warn('skip VLM reselect: target_name is empty')
            return
        if now - self._last_reselect_time < self.cooldown:
            return

        if not self._client.wait_for_service(timeout_sec=self.wait_service_timeout):
            self.get_logger().warn(
                f'VLM selector service not available: {self.service_name}'
            )
            return

        request = SelectTarget.Request()
        request.target_name = self.target_name
        request.save_debug_images = self.save_debug_images

        self._request_in_flight = True
        self._last_reselect_time = now
        future = self._client.call_async(request)
        future.add_done_callback(self._on_reselect_done)
        self.get_logger().warn(
            f'recalling VLM selector because {reason}; target={self.target_name}'
        )
        self._publish_result(
            state='reselecting',
            object_present=None,
            final_no_object=False,
            reason=reason,
        )

    def _on_reselect_done(self, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'VLM reselect call failed: {exc}')
            return

        if response.success:
            self._lost_since = None
            self._final_no_object = False
            self.get_logger().info(
                'VLM reselect succeeded | '
                f'id={response.selected_id} type={response.selected_type} '
                f'conf={response.confidence:.3f} box='
                f'[{response.x_min},{response.y_min},{response.x_max},{response.y_max}]'
            )
            self._publish_result(
                state='reselected',
                object_present=True,
                final_no_object=False,
                reason=response.message,
            )
        else:
            self.get_logger().warn(f'VLM reselect failed: {response.message}')
            if self._lost_since is not None:
                lost_for = time.monotonic() - self._lost_since
                if (
                    self.max_lost_no_object > 0.0
                    and lost_for >= self.max_lost_no_object
                    and not self._final_no_object
                ):
                    self._declare_no_object(lost_for)
                    return
            self._publish_result(
                state='reselect_failed',
                object_present=None,
                final_no_object=False,
                reason=response.message,
            )

    def _declare_no_object(self, lost_for: float) -> None:
        self._final_no_object = True
        self._request_in_flight = False
        self.get_logger().error(
            f'target={self.target_name} not found after lost_for={lost_for:.2f}s; '
            'declaring no object'
        )
        self._publish_result(
            state='no_object',
            object_present=False,
            final_no_object=True,
            reason=f'lost_for={lost_for:.2f}s >= {self.max_lost_no_object:.2f}s',
        )

    def _publish_result(
        self,
        state: str,
        object_present,
        final_no_object: bool,
        reason: str,
    ) -> None:
        lost_for = None
        if self._lost_since is not None:
            lost_for = round(time.monotonic() - self._lost_since, 3)

        payload = {
            'state': state,
            'target_name': self.target_name,
            'object_present': object_present,
            'final_no_object': bool(final_no_object),
            'lost_for_sec': lost_for,
            'lost_reselect_delay_sec': self.lost_delay,
            'max_lost_no_object_sec': self.max_lost_no_object,
            'reason': reason,
        }
        self._result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = LostReselectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
