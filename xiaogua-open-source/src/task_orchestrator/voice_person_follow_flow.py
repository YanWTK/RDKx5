#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Voice-driven interaction flow.

This ROS1 node keeps the XVF3800/ASR code untouched. It consumes the existing
ROS1 bridge outputs, turns toward the most frequent DOA observed during the
user command, records the detected person position in map, leaves task
execution as a placeholder, then navigates back to the recorded person pose.
"""

import math
import time
from collections import Counter, deque

import actionlib
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Float32, String


def normalize_deg(deg):
    return ((float(deg) + 180.0) % 360.0) - 180.0


def normalize_rad(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def clamp(value, low, high):
    return max(low, min(high, value))


def circular_mean_deg(values):
    if not values:
        return None
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return None
    return (math.degrees(math.atan2(sin_sum, cos_sum)) + 360.0) % 360.0


class VoicePersonFollowFlow:
    def __init__(self):
        rospy.init_node("voice_person_follow_flow", anonymous=False)

        self.asr_topic = rospy.get_param("~asr_topic", "/asr_command")
        self.doa_topic = rospy.get_param("~doa_topic", "/xvf3800/doa_deg")
        self.vad_topic = rospy.get_param("~vad_topic", "/xvf3800/vad")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.tts_topic = rospy.get_param("~tts_topic", "/tts_text")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.vision_frame = rospy.get_param("~vision_frame", "vision_target")

        self.doa_window_sec = float(rospy.get_param("~doa_window_sec", 6.0))
        self.doa_bin_deg = float(rospy.get_param("~doa_bin_deg", 10.0))
        self.min_doa_samples = int(rospy.get_param("~min_doa_samples", 4))
        self.use_vad_filter = bool(rospy.get_param("~use_vad_filter", True))
        self.doa_offset_deg = float(rospy.get_param("~doa_offset_deg", 0.0))
        self.doa_ccw = float(rospy.get_param("~doa_ccw", 1.0))

        self.turn_timeout_sec = float(rospy.get_param("~turn_timeout_sec", 10.0))
        self.turn_tolerance_deg = float(rospy.get_param("~turn_tolerance_deg", 6.0))
        self.turn_kp = float(rospy.get_param("~turn_kp", 1.8))
        self.min_angular_vel = float(rospy.get_param("~min_angular_vel", 0.08))
        self.max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.45))
        self.turn_direction_sign = float(rospy.get_param("~turn_direction_sign", 1.0))

        self.person_tf_timeout_sec = float(rospy.get_param("~person_tf_timeout_sec", 8.0))
        self.task_placeholder_sec = float(rospy.get_param("~task_placeholder_sec", 1.0))
        self.return_enabled = bool(rospy.get_param("~return_enabled", True))
        self.return_timeout_sec = float(rospy.get_param("~return_timeout_sec", 90.0))
        self.return_standoff_m = float(rospy.get_param("~return_standoff_m", 0.0))
        self.return_arrival_radius_m = float(rospy.get_param("~return_arrival_radius_m", 0.35))

        self._doa_samples = deque()
        self._last_vad = False
        self._busy = False
        self._last_person_pose = None

        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=10)
        self.state_pub = rospy.Publisher("/voice_flow/state", String, queue_size=10, latch=True)
        self.person_pose_pub = rospy.Publisher(
            "/voice_flow/person_pose_map",
            PoseStamped,
            queue_size=1,
            latch=True,
        )

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Subscriber(self.doa_topic, Float32, self._on_doa, queue_size=50)
        rospy.Subscriber(self.vad_topic, Bool, self._on_vad, queue_size=50)
        rospy.Subscriber(self.asr_topic, String, self._on_asr, queue_size=10)

        rospy.loginfo(
            "voice_person_follow_flow started: asr=%s doa=%s vad=%s map=%s base=%s "
            "vision=%s doa_window=%.1fs bin=%.1fdeg return_enabled=%s",
            self.asr_topic,
            self.doa_topic,
            self.vad_topic,
            self.map_frame,
            self.base_frame,
            self.vision_frame,
            self.doa_window_sec,
            self.doa_bin_deg,
            self.return_enabled,
        )
        self._publish_state("idle")

    def _on_vad(self, msg):
        self._last_vad = bool(msg.data)

    def _on_doa(self, msg):
        now = time.monotonic()
        if self.use_vad_filter and not self._last_vad:
            self._trim_doa_samples(now)
            return
        self._doa_samples.append((now, float(msg.data)))
        self._trim_doa_samples(now)

    def _on_asr(self, msg):
        command = msg.data.strip()
        if not command:
            return
        if self._busy:
            rospy.logwarn("flow is busy, ignore ASR command: %s", command)
            return

        self._busy = True
        try:
            self._run_flow(command)
        finally:
            self._stop_base()
            self._busy = False
            self._publish_state("idle")

    def _run_flow(self, command):
        rospy.loginfo("ASR command received: %s", command)
        self._publish_state("received_command")

        doa_deg = self._most_frequent_doa()
        if doa_deg is None:
            rospy.logwarn("not enough DOA samples; skip turn, command=%s", command)
            self._say("我听到了，但没有稳定的声源方向")
        else:
            rospy.loginfo("selected DOA: %.1f deg", doa_deg)
            self._publish_state("turning_to_speaker")
            self._turn_by_relative_angle(doa_deg)

        self._publish_state("recording_person_position")
        person_pose = self._wait_person_pose_in_map()
        if person_pose is None:
            rospy.logwarn("person map pose was not available after turn")
            self._say("我没有在地图上记录到你的位置")
        else:
            self._last_person_pose = person_pose
            self.person_pose_pub.publish(person_pose)
            rospy.loginfo(
                "recorded person pose in map: x=%.3f y=%.3f",
                person_pose.pose.position.x,
                person_pose.pose.position.y,
            )
            self._say("我已经记录你的位置")

        self._publish_state("task_placeholder")
        self._execute_user_command_placeholder(command)

        if self.return_enabled and self._last_person_pose is not None:
            self._publish_state("returning_to_person")
            self._navigate_back_to_person(self._last_person_pose)

    def _trim_doa_samples(self, now):
        cutoff = now - max(self.doa_window_sec, 0.5)
        while self._doa_samples and self._doa_samples[0][0] < cutoff:
            self._doa_samples.popleft()

    def _most_frequent_doa(self):
        now = time.monotonic()
        self._trim_doa_samples(now)
        samples = [raw for _, raw in self._doa_samples]
        if len(samples) < self.min_doa_samples:
            return None

        corrected = [self._correct_doa(raw) for raw in samples]
        bin_size = max(self.doa_bin_deg, 1.0)
        bins = Counter(int(round((deg % 360.0) / bin_size)) for deg in corrected)
        winner, count = bins.most_common(1)[0]
        winner_center = (winner * bin_size) % 360.0
        in_winner = [
            deg for deg in corrected
            if abs(normalize_deg(deg - winner_center)) <= bin_size
        ]
        mean = circular_mean_deg(in_winner) if in_winner else winner_center
        selected = normalize_deg(mean)
        rospy.loginfo(
            "DOA histogram selected %.1fdeg from %d/%d samples",
            selected,
            count,
            len(samples),
        )
        return selected

    def _correct_doa(self, raw_deg):
        return normalize_deg(float(raw_deg) * self.doa_ccw + self.doa_offset_deg)

    def _lookup_yaw(self):
        trans, rot = self.tf_listener.lookupTransform(
            self.map_frame,
            self.base_frame,
            rospy.Time(0),
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)
        return float(trans[0]), float(trans[1]), yaw

    def _turn_by_relative_angle(self, doa_deg):
        try:
            _, _, start_yaw = self._lookup_yaw()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("cannot lookup base yaw; skip turn: %s", exc)
            return False

        target_yaw = normalize_rad(start_yaw + math.radians(doa_deg))
        rate = rospy.Rate(20)
        deadline = time.monotonic() + self.turn_timeout_sec
        ok = False

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                _, _, current_yaw = self._lookup_yaw()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue

            err = normalize_rad(target_yaw - current_yaw)
            err_deg = math.degrees(err)
            if abs(err_deg) <= self.turn_tolerance_deg:
                ok = True
                break

            angular = clamp(
                self.turn_kp * err,
                -self.max_angular_vel,
                self.max_angular_vel,
            )
            if abs(angular) < self.min_angular_vel:
                angular = math.copysign(self.min_angular_vel, angular)
            self._publish_turn_cmd(angular * self.turn_direction_sign)
            rate.sleep()

        self._stop_base()
        if ok:
            rospy.loginfo("turn completed: target relative %.1fdeg", doa_deg)
        else:
            rospy.logwarn("turn timeout: target relative %.1fdeg", doa_deg)
        return ok

    def _wait_person_pose_in_map(self):
        deadline = time.monotonic() + self.person_tf_timeout_sec
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    self.map_frame,
                    self.vision_frame,
                    rospy.Time(0),
                )
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.map_frame
                pose.pose.position.x = float(trans[0])
                pose.pose.position.y = float(trans[1])
                pose.pose.position.z = float(trans[2])
                pose.pose.orientation.x = float(rot[0])
                pose.pose.orientation.y = float(rot[1])
                pose.pose.orientation.z = float(rot[2])
                pose.pose.orientation.w = float(rot[3])
                return pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
        return None

    def _execute_user_command_placeholder(self, command):
        rospy.loginfo("task execution placeholder, command left unhandled: %s", command)
        if self.task_placeholder_sec > 0:
            rospy.sleep(self.task_placeholder_sec)

    def _navigate_back_to_person(self, person_pose):
        if not self.move_base.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("move_base server is not available; cannot return to person")
            self._say("导航服务器没有响应，暂时不能回到你的位置")
            return False

        goal_x = person_pose.pose.position.x
        goal_y = person_pose.pose.position.y
        goal_yaw = 0.0

        if self.return_standoff_m > 0.0:
            try:
                robot_x, robot_y, _ = self._lookup_yaw()
                dx = goal_x - robot_x
                dy = goal_y - robot_y
                dist = math.hypot(dx, dy)
                if dist > self.return_standoff_m:
                    goal_x -= self.return_standoff_m * dx / dist
                    goal_y -= self.return_standoff_m * dy / dist
                goal_yaw = math.atan2(person_pose.pose.position.y - goal_y, person_pose.pose.position.x - goal_x)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        rospy.loginfo("returning to recorded person pose: x=%.3f y=%.3f", goal_x, goal_y)
        self._say("我现在回到你的位置")
        self.move_base.send_goal(goal)
        rate = rospy.Rate(10)
        deadline = time.monotonic() + self.return_timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.return_arrival_radius_m > 0.0:
                try:
                    robot_x, robot_y, _ = self._lookup_yaw()
                    dist_to_person = math.hypot(
                        person_pose.pose.position.x - robot_x,
                        person_pose.pose.position.y - robot_y,
                    )
                    if dist_to_person <= self.return_arrival_radius_m:
                        self.move_base.cancel_goal()
                        self._stop_base()
                        rospy.loginfo(
                            "returned within arrival radius: dist=%.3fm radius=%.3fm",
                            dist_to_person,
                            self.return_arrival_radius_m,
                        )
                        self._say("我回来了")
                        return True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

            state = self.move_base.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("returned to recorded person pose")
                self._say("我回来了")
                return True
            if state in (
                actionlib.GoalStatus.PREEMPTED,
                actionlib.GoalStatus.ABORTED,
                actionlib.GoalStatus.REJECTED,
                actionlib.GoalStatus.RECALLED,
                actionlib.GoalStatus.LOST,
            ):
                rospy.logwarn("return navigation failed, state=%s", state)
                self._say("我没有成功回到你的位置")
                return False

            rate.sleep()

        if not rospy.is_shutdown():
            self.move_base.cancel_goal()
            rospy.logwarn("return navigation timeout")
            self._say("回到你的位置超时了")
        return False

    def _publish_turn_cmd(self, angular_z):
        cmd = Twist()
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    def _say(self, text):
        if not text:
            return
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)

    def _publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        VoicePersonFollowFlow().run()
    except rospy.ROSInterruptException:
        pass
