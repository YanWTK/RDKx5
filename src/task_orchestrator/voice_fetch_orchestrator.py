#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Voice-driven fetch task orchestrator.

This node owns the high-level task flow. It keeps the existing perception,
tracking, auto-aim, navigation, and arm nodes as separate capabilities.
"""

import json
import math
import os
import tempfile
import threading
import time
import urllib.error
import urllib.request
from collections import Counter, deque

import actionlib
import rospy
import rospkg
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger


# ===== 常用调参区：你平时主要改这里 =====
# 普通抓取：让目标对准画面中间。像素坐标基于 640x480 图像。
GRASP_PIXEL_X = 320
GRASP_TOL_PX = 10

# 相对放置：例如“把快递放到矿泉水左/右边”。
# 放到左边时，把参照物对到画面右侧；放到右边时，把参照物对到画面左侧。
PLACE_LEFT_PIXEL_X = 520
PLACE_RIGHT_PIXEL_X = 120
PLACE_TOL_PX = 25
GRASP_TARGET_DIST_CM = 12.0
GRASP_STAGE1_DIST_CM = 16.0
GRASP_TOL_DIST_STAGE1 = 2.0
GRASP_TOL_DIST_FINAL = 1.5
GRASP_TOL_YAW = 5.0
GRASP_TOL_PIXEL_ROUGH = 30
GRASP_MIN_VEL_Y = 0.05
GRASP_MIN_VEL_Z = 0.10

# 放置对准：默认比抓取更保守，可以通过启动脚本环境变量继续调。
PLACE_TARGET_DIST_CM = 18.0
PLACE_STAGE1_DIST_CM = 24.0
PLACE_TOL_DIST_STAGE1 = 2.5
PLACE_TOL_DIST_FINAL = 2.0
PLACE_TOL_YAW = 6.0
PLACE_TOL_PIXEL_ROUGH = 40
PLACE_MIN_VEL_Y = 0.04
PLACE_MIN_VEL_Z = 0.08

CUSTOM_YOLO_CLASSES = [
    "person",
    "cell phone",
    "mouse",
    "remote",
    "book",
    "bottle",
    "cup",
    "bowl",
    "apple",
    "banana",
    "teddy bear",
    "bag_wrapper",
    "box",
]


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


class VoiceFetchOrchestrator:
    def __init__(self):
        rospy.init_node("voice_fetch_orchestrator", anonymous=False)

        self.asr_topic = rospy.get_param("~asr_topic", "/asr_command")
        self.manual_command_topic = rospy.get_param("~manual_command_topic", "/voice_fetch/command")
        self.doa_topic = rospy.get_param("~doa_topic", "/xvf3800/doa_deg")
        self.vad_topic = rospy.get_param("~vad_topic", "/xvf3800/vad")
        self.tts_playing_topic = rospy.get_param("~tts_playing_topic", "/tts_playing")
        self.doa_session_topic = rospy.get_param(
            "~doa_session_topic", "/voice_fetch/doa_session"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.tts_topic = rospy.get_param("~tts_topic", "/tts_text")

        self.task_understanding_query_topic = rospy.get_param(
            "~task_understanding_query_topic", "/task_understanding/query"
        )
        self.task_understanding_result_topic = rospy.get_param(
            "~task_understanding_result_topic", "/task_understanding/result"
        )
        self.memory_query_topic = rospy.get_param("~memory_query_topic", "/object_memory/query")
        self.memory_query_result_topic = rospy.get_param(
            "~memory_query_result_topic", "/object_memory/query_result"
        )
        self.target_confirm_cmd_topic = rospy.get_param(
            "~target_confirm_cmd_topic", "/target_confirm/confirm_cmd"
        )
        self.target_confirm_result_topic = rospy.get_param(
            "~target_confirm_result_topic", "/target_confirm/result"
        )
        self.memory_select_cmd_topic = rospy.get_param(
            "~memory_select_cmd_topic", "/memory_target_selector/select_cmd"
        )
        self.memory_select_result_topic = rospy.get_param(
            "~memory_select_result_topic", "/memory_target_selector/result"
        )
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/object_tracker/status")
        self.align_success_topic = rospy.get_param("~align_success_topic", "/red_align_success")

        self.align_service = rospy.get_param("~align_service", "/enable_redalign")
        self.grasp_service = rospy.get_param("~grasp_service", "/execute_grasp")
        self.retract_service = rospy.get_param("~retract_service", "/execute_retract")
        self.placement_pre_align_service = rospy.get_param(
            "~placement_pre_align_service", "/execute_retract_140"
        )
        self.placement_pre_align_settle_sec = float(
            rospy.get_param("~placement_pre_align_settle_sec", 1.5)
        )
        self.release_service = rospy.get_param("~release_service", "/execute_release")
        self.arm_standby_service = rospy.get_param("~arm_standby_service", "/arm_init_pose")
        self.arm_ready_service = rospy.get_param("~arm_ready_service", "/arm_grasp_ready")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.vision_frame = rospy.get_param("~vision_frame", "vision_target")
        pkg_path = rospkg.RosPack().get_path("nav_pkg")
        self.speech_profile_path = rospy.get_param("~speech_profile_path", "")
        if not self.speech_profile_path:
            self.speech_profile_path = pkg_path + "/config/voice_persona.json"
        self.speech_profile_name = rospy.get_param("~speech_profile", "")
        self._speech_profile_mtime = None
        self._speech_profile = {}
        self.enable_llm_speech = _as_bool(rospy.get_param("~enable_llm_speech", False))
        self.llm_speech_model = rospy.get_param("~llm_speech_model", "qwen3.6-flash")
        self.llm_speech_base_url = rospy.get_param(
            "~llm_speech_base_url",
            "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        self.llm_speech_api_key_env = rospy.get_param(
            "~llm_speech_api_key_env", "DASHSCOPE_API_KEY"
        )
        self.llm_speech_timeout_sec = float(
            rospy.get_param("~llm_speech_timeout_sec", 2.5)
        )
        self.llm_speech_temperature = float(
            rospy.get_param("~llm_speech_temperature", 0.9)
        )
        self.patrol_points_path = rospy.get_param("~patrol_points_path", "")
        if not self.patrol_points_path:
            self.patrol_points_path = pkg_path + "/config/patrol_points.json"

        self.doa_window_sec = float(rospy.get_param("~doa_window_sec", 6.0))
        self.doa_bin_deg = float(rospy.get_param("~doa_bin_deg", 10.0))
        self.min_doa_samples = int(rospy.get_param("~min_doa_samples", 4))
        self.doa_min_winner_ratio = float(
            rospy.get_param("~doa_min_winner_ratio", 0.45)
        )
        self.doa_accept_window_deg = float(
            rospy.get_param("~doa_accept_window_deg", 25.0)
        )
        self.doa_cluster_window_deg = float(
            rospy.get_param("~doa_cluster_window_deg", 30.0)
        )
        self.doa_ignore_edge_sec = float(
            rospy.get_param("~doa_ignore_edge_sec", 0.2)
        )
        self.use_vad_filter = _as_bool(rospy.get_param("~use_vad_filter", True))
        self.doa_offset_deg = float(rospy.get_param("~doa_offset_deg", 45.0))
        self.doa_ccw = float(rospy.get_param("~doa_ccw", 1.0))

        self.turn_timeout_sec = float(rospy.get_param("~turn_timeout_sec", 10.0))
        self.turn_tolerance_deg = float(rospy.get_param("~turn_tolerance_deg", 6.0))
        self.turn_kp = float(rospy.get_param("~turn_kp", 1.8))
        self.min_angular_vel = float(rospy.get_param("~min_angular_vel", 0.08))
        self.max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.45))
        self.turn_direction_sign = float(rospy.get_param("~turn_direction_sign", 1.0))

        self.person_tf_timeout_sec = float(rospy.get_param("~person_tf_timeout_sec", 8.0))
        self.person_retry_enabled = _as_bool(
            rospy.get_param("~person_retry_enabled", True)
        )
        self.person_retry_listen_timeout_sec = float(
            rospy.get_param("~person_retry_listen_timeout_sec", 6.0)
        )
        self.person_retry_min_listen_sec = float(
            rospy.get_param("~person_retry_min_listen_sec", 1.2)
        )
        self.person_retry_prompt = rospy.get_param(
            "~person_retry_prompt", "我没看到你，你再说一句我在这里"
        )
        self.use_robot_pose_if_no_person = _as_bool(
            rospy.get_param("~use_robot_pose_if_no_person", True)
        )
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout_sec", 90.0))
        self.return_timeout_sec = float(rospy.get_param("~return_timeout_sec", 90.0))
        self.return_standoff_m = float(rospy.get_param("~return_standoff_m", 0.5))
        self.return_arrival_radius_m = float(rospy.get_param("~return_arrival_radius_m", 0.35))
        self.deliver_person_backup_m = float(rospy.get_param("~deliver_person_backup_m", 0.5))
        self.deliver_person_backup_speed = float(rospy.get_param("~deliver_person_backup_speed", 0.12))
        self.deliver_person_search_step_deg = float(rospy.get_param("~deliver_person_search_step_deg", 30.0))
        self.deliver_person_search_angular_speed = float(
            rospy.get_param("~deliver_person_search_angular_speed", 0.18)
        )
        self.deliver_person_search_max_deg = float(
            rospy.get_param("~deliver_person_search_max_deg", 360.0)
        )

        self.memory_query_timeout_sec = float(rospy.get_param("~memory_query_timeout_sec", 22.0))
        self.task_understanding_timeout_sec = float(
            rospy.get_param("~task_understanding_timeout_sec", 22.0)
        )
        self.enable_task_understanding = _as_bool(
            rospy.get_param("~enable_task_understanding", True)
        )
        self.task_memory_path = rospy.get_param(
            "~task_memory_path", "/opt/xiaogua/data/robopilot_memory/task_memory.json"
        )
        self.task_memory_context_count = int(
            rospy.get_param("~task_memory_context_count", 2)
        )
        self.task_memory_max_records = int(
            rospy.get_param("~task_memory_max_records", 50)
        )
        self.target_confirm_timeout_sec = float(rospy.get_param("~target_confirm_timeout_sec", 30.0))
        self.memory_select_timeout_sec = float(rospy.get_param("~memory_select_timeout_sec", 30.0))
        self.tracker_lock_timeout_sec = float(rospy.get_param("~tracker_lock_timeout_sec", 8.0))
        self.align_timeout_sec = float(rospy.get_param("~align_timeout_sec", 50.0))
        self.align_min_success_delay_sec = float(
            rospy.get_param("~align_min_success_delay_sec", 1.0)
        )
        self.service_timeout_sec = float(rospy.get_param("~service_timeout_sec", 8.0))
        self.capture_count = int(rospy.get_param("~capture_count", 5))
        self.auto_aim_target_pixel_param = rospy.get_param(
            "~auto_aim_target_pixel_param", "/auto_aim_target_pixel_x"
        )
        self.auto_aim_tol_pixel_fine_param = rospy.get_param(
            "~auto_aim_tol_pixel_fine_param", "/auto_aim_tol_pixel_fine"
        )
        self.auto_aim_target_dist_cm_param = rospy.get_param(
            "~auto_aim_target_dist_cm_param", "/auto_aim_target_dist_cm"
        )
        self.auto_aim_stage1_dist_cm_param = rospy.get_param(
            "~auto_aim_stage1_dist_cm_param", "/auto_aim_stage1_dist_cm"
        )
        self.auto_aim_tol_dist_stage1_param = rospy.get_param(
            "~auto_aim_tol_dist_stage1_param", "/auto_aim_tol_dist_stage1"
        )
        self.auto_aim_tol_dist_final_param = rospy.get_param(
            "~auto_aim_tol_dist_final_param", "/auto_aim_tol_dist_final"
        )
        self.auto_aim_tol_yaw_param = rospy.get_param(
            "~auto_aim_tol_yaw_param", "/auto_aim_tol_yaw"
        )
        self.auto_aim_tol_pixel_rough_param = rospy.get_param(
            "~auto_aim_tol_pixel_rough_param", "/auto_aim_tol_pixel_rough"
        )
        self.auto_aim_min_vel_y_param = rospy.get_param(
            "~auto_aim_min_vel_y_param", "/auto_aim_min_vel_y"
        )
        self.auto_aim_min_vel_z_param = rospy.get_param(
            "~auto_aim_min_vel_z_param", "/auto_aim_min_vel_z"
        )
        self.auto_aim_center_pixel_x = int(
            rospy.get_param("~auto_aim_center_pixel_x", GRASP_PIXEL_X)
        )
        self.auto_aim_center_tol_pixel_fine = int(
            rospy.get_param("~auto_aim_center_tol_pixel_fine", GRASP_TOL_PX)
        )
        self.auto_aim_grasp_params = {
            "target_dist_cm": float(rospy.get_param("~auto_aim_grasp_target_dist_cm", GRASP_TARGET_DIST_CM)),
            "stage1_dist_cm": float(rospy.get_param("~auto_aim_grasp_stage1_dist_cm", GRASP_STAGE1_DIST_CM)),
            "tol_dist_stage1": float(rospy.get_param("~auto_aim_grasp_tol_dist_stage1", GRASP_TOL_DIST_STAGE1)),
            "tol_dist_final": float(rospy.get_param("~auto_aim_grasp_tol_dist_final", GRASP_TOL_DIST_FINAL)),
            "tol_yaw": float(rospy.get_param("~auto_aim_grasp_tol_yaw", GRASP_TOL_YAW)),
            "tol_pixel_rough": int(rospy.get_param("~auto_aim_grasp_tol_pixel_rough", GRASP_TOL_PIXEL_ROUGH)),
            "min_vel_y": float(rospy.get_param("~auto_aim_grasp_min_vel_y", GRASP_MIN_VEL_Y)),
            "min_vel_z": float(rospy.get_param("~auto_aim_grasp_min_vel_z", GRASP_MIN_VEL_Z)),
        }
        self.placement_tol_pixel_fine = int(
            rospy.get_param("~placement_tol_pixel_fine", PLACE_TOL_PX)
        )
        self.placement_left_reference_pixel_x = int(
            rospy.get_param("~placement_left_reference_pixel_x", PLACE_LEFT_PIXEL_X)
        )
        self.placement_right_reference_pixel_x = int(
            rospy.get_param("~placement_right_reference_pixel_x", PLACE_RIGHT_PIXEL_X)
        )
        self.auto_aim_place_params = {
            "target_dist_cm": float(rospy.get_param("~auto_aim_place_target_dist_cm", PLACE_TARGET_DIST_CM)),
            "stage1_dist_cm": float(rospy.get_param("~auto_aim_place_stage1_dist_cm", PLACE_STAGE1_DIST_CM)),
            "tol_dist_stage1": float(rospy.get_param("~auto_aim_place_tol_dist_stage1", PLACE_TOL_DIST_STAGE1)),
            "tol_dist_final": float(rospy.get_param("~auto_aim_place_tol_dist_final", PLACE_TOL_DIST_FINAL)),
            "tol_yaw": float(rospy.get_param("~auto_aim_place_tol_yaw", PLACE_TOL_YAW)),
            "tol_pixel_rough": int(rospy.get_param("~auto_aim_place_tol_pixel_rough", PLACE_TOL_PIXEL_ROUGH)),
            "min_vel_y": float(rospy.get_param("~auto_aim_place_min_vel_y", PLACE_MIN_VEL_Y)),
            "min_vel_z": float(rospy.get_param("~auto_aim_place_min_vel_z", PLACE_MIN_VEL_Z)),
        }
        self.dry_run = _as_bool(rospy.get_param("~dry_run", False))
        self.stop_after_auto_aim = _as_bool(
            rospy.get_param("~stop_after_auto_aim", False)
        )

        self._doa_samples = deque()
        self._doa_lock = threading.RLock()
        self._doa_session_id = None
        self._doa_recording = False
        self._latched_doa_samples = None
        self._last_vad = False
        self._tts_playing = False
        self._busy = False
        self._held_object = self._load_last_held_object()
        self._patrol_points = self._load_patrol_points()

        self._memory_condition = threading.Condition()
        self._understanding_condition = threading.Condition()
        self._target_condition = threading.Condition()
        self._select_condition = threading.Condition()
        self._tracker_condition = threading.Condition()
        self._align_condition = threading.Condition()
        self._memory_result = None
        self._understanding_result = None
        self._target_result = None
        self._select_result = None
        self._tracker_status = None
        self._align_success_seen = False
        self._align_success_time = 0.0
        self._align_active_since = 0.0
        self._pending_memory_request_id = ""
        self._pending_understanding_request_id = ""

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=10)
        self.state_pub = rospy.Publisher("/voice_fetch/state", String, queue_size=10, latch=True)
        self.persona_status_pub = rospy.Publisher(
            "/voice_persona/status", String, queue_size=10, latch=True
        )
        self.person_pose_pub = rospy.Publisher(
            "/voice_fetch/person_pose_map", PoseStamped, queue_size=1, latch=True
        )

        self.task_understanding_pub = rospy.Publisher(
            self.task_understanding_query_topic, String, queue_size=10
        )
        self.memory_query_pub = rospy.Publisher(self.memory_query_topic, String, queue_size=10)
        self.target_confirm_pub = rospy.Publisher(self.target_confirm_cmd_topic, String, queue_size=10)
        self.memory_select_pub = rospy.Publisher(self.memory_select_cmd_topic, String, queue_size=10)

        rospy.Subscriber(self.doa_topic, Float32, self._on_doa, queue_size=50)
        rospy.Subscriber(self.vad_topic, Bool, self._on_vad, queue_size=50)
        rospy.Subscriber(
            self.tts_playing_topic, Bool, self._on_tts_playing, queue_size=10
        )
        rospy.Subscriber(
            self.doa_session_topic, String, self._on_doa_session, queue_size=10
        )
        rospy.Subscriber(self.asr_topic, String, self._on_command, queue_size=10)
        rospy.Subscriber(self.manual_command_topic, String, self._on_command, queue_size=10)
        rospy.Subscriber("/voice_cmd", String, self._on_command, queue_size=10)
        rospy.Subscriber("/voice_persona/set", String, self._on_voice_persona_set, queue_size=10)
        rospy.Subscriber(
            self.task_understanding_result_topic,
            String,
            self._on_understanding_result,
            queue_size=10,
        )
        rospy.Subscriber(self.memory_query_result_topic, String, self._on_memory_result, queue_size=10)
        rospy.Subscriber(self.target_confirm_result_topic, String, self._on_target_result, queue_size=10)
        rospy.Subscriber(self.memory_select_result_topic, String, self._on_select_result, queue_size=10)
        rospy.Subscriber(self.tracker_status_topic, String, self._on_tracker_status, queue_size=10)
        rospy.Subscriber(self.align_success_topic, Bool, self._on_align_success, queue_size=10)

        rospy.loginfo(
            "voice_fetch_orchestrator started | asr=%s manual=%s points=%d dry_run=%s stop_after_auto_aim=%s",
            self.asr_topic,
            self.manual_command_topic,
            len(self._patrol_points),
            self.dry_run,
            self.stop_after_auto_aim,
        )
        self._publish_state("idle")
        self._publish_persona_status("ready")

    def _on_voice_persona_set(self, msg):
        raw = str(msg.data or "").strip()
        if not raw:
            self._publish_persona_status("error", reason="empty request")
            return
        try:
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                payload = {"profile": str(payload)}
        except Exception:
            payload = {"custom_style": raw}

        try:
            data = self._load_persona_config()
            profiles = data.setdefault("profiles", {})
            requested = self._clean_persona_text(
                payload.get("profile")
                or payload.get("active_profile")
                or payload.get("mode")
                or ""
            )
            custom_style = self._clean_persona_text(
                payload.get("custom_style")
                or payload.get("style")
                or payload.get("description")
                or ""
            )

            if custom_style:
                base_name = self._clean_persona_text(
                    payload.get("base_profile") or data.get("active_profile") or "default"
                )
                base = profiles.get(base_name) or profiles.get("default") or {}
                profiles["custom"] = self._build_custom_profile(
                    custom_style,
                    base,
                    data.get("custom_profile_template"),
                )
                data["active_profile"] = "custom"
                changed = "custom"
            else:
                profile = requested or "default"
                if profile not in profiles:
                    self._publish_persona_status("error", reason="unknown profile: %s" % profile)
                    return
                data["active_profile"] = profile
                changed = profile

            data["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            self._write_persona_config(data)
            self._speech_profile_mtime = None
            self._publish_persona_status("ok", active_profile=changed, custom_style=custom_style)
            rospy.loginfo("voice persona set: active=%s custom=%s", changed, bool(custom_style))
        except Exception as exc:
            rospy.logwarn("failed to set voice persona: %s", exc)
            self._publish_persona_status("error", reason=str(exc))

    def _on_vad(self, msg):
        with self._doa_lock:
            self._last_vad = bool(msg.data)

    def _on_tts_playing(self, msg):
        with self._doa_lock:
            self._tts_playing = bool(msg.data)

    def _on_doa(self, msg):
        now = time.monotonic()
        with self._doa_lock:
            if self.use_vad_filter and not self._last_vad:
                if not self._doa_recording:
                    self._trim_doa_samples(now)
                return
            self._doa_samples.append((now, float(msg.data)))
            if not self._doa_recording:
                self._trim_doa_samples(now)

    def _on_doa_session(self, msg):
        try:
            payload = json.loads(msg.data)
            event = str(payload.get("event") or "").strip()
            session_id = int(payload.get("session_id"))
        except Exception as exc:
            rospy.logwarn("ignore invalid DOA session event: %s", exc)
            return

        with self._doa_lock:
            if event == "wake":
                self._doa_session_id = session_id
                self._doa_recording = False
                self._doa_samples.clear()
                self._latched_doa_samples = None
                rospy.loginfo("DOA session %d opened", session_id)
                return

            if event == "recording_start" and self._doa_session_id is None:
                # Recover if the wake event was emitted before rosbridge or this
                # node became ready. Recording start is authoritative.
                self._doa_session_id = session_id
                self._doa_samples.clear()
                self._latched_doa_samples = None

            if self._doa_session_id != session_id:
                rospy.logwarn(
                    "ignore stale DOA session event: event=%s session=%d active=%s",
                    event,
                    session_id,
                    self._doa_session_id,
                )
                return

            if event == "recording_start":
                self._doa_recording = True
                self._doa_samples.clear()
                self._latched_doa_samples = None
                rospy.loginfo("DOA session %d recording started", session_id)
                return

            if event == "recording_end":
                self._doa_recording = False
                self._latched_doa_samples = list(self._doa_samples)
                rospy.loginfo(
                    "DOA session %d latched %d samples",
                    session_id,
                    len(self._latched_doa_samples),
                )
                return

            if event == "abort":
                self._doa_recording = False
                self._doa_samples.clear()
                self._latched_doa_samples = None
                self._doa_session_id = None
                rospy.loginfo("DOA session %d aborted", session_id)
                return

            rospy.logwarn("ignore unknown DOA session event: %s", event)

    def _on_command(self, msg):
        command = msg.data.strip()
        if not command:
            return
        if self._busy:
            rospy.logwarn("fetch flow is busy, ignore command: %s", command)
            self._say_key("busy", "我正在执行上一个任务")
            return
        self._busy = True
        threading.Thread(target=self._run_flow, args=(command,), daemon=True).start()

    def _run_flow(self, command):
        understanding = {}
        task_result = "failed"
        task_reason = "任务未完成"
        try:
            rospy.loginfo("fetch command received: %s", command)
            self._publish_state("received_command")

            understanding = self._understand_command(command) or {}
            intent = str(understanding.get("intent") or "unknown").strip()
            if intent == "fetch_object":
                intent = "fetch_to_speaker"
            rospy.loginfo(
                "task understanding route: intent=%s payload=%s",
                intent,
                json.dumps(understanding, ensure_ascii=False, sort_keys=True),
            )

            tts_text = str(understanding.get("tts_text") or "").strip()
            if intent == "chat":
                self._publish_state("chat")
                if self._is_memory_location_question(command, understanding):
                    if tts_text:
                        self._say(tts_text)
                    if self._answer_memory_location_question(command, understanding):
                        task_result = "success"
                        task_reason = "记忆位置查询完成"
                    else:
                        task_reason = "记忆位置查询失败"
                    return
                self._say(tts_text or "可以，我陪你聊聊")
                task_result = "success"
                task_reason = "闲聊完成"
                return
            if intent == "unknown" and self._has_action_plan(understanding):
                if tts_text:
                    self._say(tts_text)
                if self._run_action_plan(command, understanding):
                    task_result = "success"
                    task_reason = "计划执行完成"
                else:
                    task_reason = "复杂计划未完成"
                return
            if intent == "come_to_speaker":
                self._publish_state("come_to_speaker")
                if tts_text:
                    self._say(tts_text)
                if self._run_come_to_speaker_task():
                    task_result = "success"
                    task_reason = "已导航到说话人附近"
                else:
                    task_reason = "没有成功导航到说话人附近"
                return
            if not self._prepare_arm_before_task():
                self._say("机械臂没有回到初始状态，任务取消")
                task_reason = "机械臂没有回到初始状态"
                return
            if intent == "navigate_to":
                self._publish_state("navigate_task")
                if tts_text:
                    self._say(tts_text)
                if self._run_navigation_task(understanding):
                    task_result = "success"
                    task_reason = "导航完成"
                else:
                    task_reason = "导航失败"
                return
            if intent == "task_chain":
                self._publish_state("task_chain")
                if tts_text:
                    self._say(tts_text)
                if self._run_task_chain(understanding):
                    task_result = "success"
                    task_reason = "任务链完成"
                else:
                    task_reason = "任务链未完成"
                return
            if intent == "transfer_object":
                self._publish_state("transfer_object")
                if tts_text:
                    self._say(tts_text)
                if self._is_relative_placement_task(understanding):
                    if self._run_relative_placement_task(command, understanding):
                        task_result = "success"
                        task_reason = "相对放置完成"
                    else:
                        task_reason = "相对放置失败"
                    return
                if self._run_transfer_task(command, understanding):
                    task_result = "success"
                    task_reason = "搬运完成"
                else:
                    task_reason = "搬运失败"
                return
            if intent == "deliver_to_person":
                self._publish_state("deliver_to_person")
                if tts_text:
                    self._say(tts_text)
                if self._run_deliver_to_person_task(command, understanding):
                    task_result = "success"
                    task_reason = "已拿到物品并导航到目标人物附近"
                else:
                    task_reason = "人物交付失败"
                return
            if intent != "fetch_to_speaker":
                rospy.logwarn("unknown or unsupported task intent: %s", understanding)
                self._say(tts_text or "我还不确定该执行什么任务")
                task_reason = "无法确定要执行的任务"
                return

            self._say_key("received_command", "收到，我先确认你的位置")

            person_pose = self._record_speaker_pose_after_stable_doa("speaker")
            if person_pose is None:
                self._say_key("no_person_pose", "我没有记录到你的位置，任务取消")
                task_reason = "没有记录到说话人位置"
                return
            self.person_pose_pub.publish(person_pose)
            rospy.loginfo(
                "person pose published: frame=%s x=%.3f y=%.3f z=%.3f q=(%.3f %.3f %.3f %.3f)",
                person_pose.header.frame_id,
                person_pose.pose.position.x,
                person_pose.pose.position.y,
                person_pose.pose.position.z,
                person_pose.pose.orientation.x,
                person_pose.pose.orientation.y,
                person_pose.pose.orientation.z,
                person_pose.pose.orientation.w,
            )

            if tts_text:
                self._say(tts_text)

            self._publish_state("query_memory")
            memory = self._query_memory(command, understanding)
            fallback_target_obj = None
            if not memory or not memory.get("success"):
                reason = memory.get("reason") if isinstance(memory, dict) else "记忆查询失败"
                rospy.logwarn("memory query failed: %s", reason)
                memory = self._fallback_memory_from_likely_locations(command, understanding)
                if not memory:
                    self._say_key("memory_not_found", "我没有在记忆里找到这个物品")
                    task_reason = "记忆库和现场候选地点都没有找到目标"
                    return
                fallback_target_obj = memory.get("target_obj")

            target_id = str(memory.get("target_id") or "")
            target_name = str(memory.get("target_name") or "")
            point_id = str(memory.get("point_id") or "")
            rospy.loginfo(
                "memory query selected: target_id=%s target_name=%s point_id=%s raw=%s",
                target_id,
                target_name,
                point_id,
                json.dumps(memory, ensure_ascii=False, sort_keys=True),
            )
            if not target_id or not point_id:
                self._say_key("memory_incomplete", "目标记忆不完整，不能导航")
                task_reason = "目标记忆不完整"
                return

            self._say_key(
                "navigate_to_memory",
                "我去记忆中的位置找{target_name}",
                target_name=target_name or "目标",
            )
            self._publish_state("navigate_to_object_point")
            if not self._navigate_to_point(point_id):
                self._say_key("nav_failed", "我没有到达物品所在点位")
                task_reason = "没有到达物品所在点位"
                return

            # Camera/YOLO are not needed while understanding, querying memory,
            # or navigating. Enable the vision relay only after arrival and hold
            # it until the final idle state for confirmation and tracking.
            self._publish_state("fetch_vision_active")
            self._publish_state("confirm_target")
            confirm = self._confirm_target(
                target_id,
                target_name,
                command,
                target_obj=fallback_target_obj,
            )
            rospy.loginfo(
                "target confirm result: %s",
                json.dumps(confirm or {}, ensure_ascii=False, sort_keys=True),
            )
            if not confirm or not confirm.get("target_found"):
                reason = confirm.get("reason") if isinstance(confirm, dict) else "目标确认失败"
                rospy.logwarn("target confirm failed: %s", reason)
                self._say_key("target_not_confirmed", "我到了点位，但没有确认到目标")
                task_reason = "到达点位后没有确认到目标"
                return

            self._publish_state("select_tracking_target")
            select = self._select_tracking_target(
                target_id,
                target_name,
                command,
                target_obj=fallback_target_obj,
            )
            rospy.loginfo(
                "tracking target select result: %s",
                json.dumps(select or {}, ensure_ascii=False, sort_keys=True),
            )
            if not select or not select.get("success"):
                reason = select.get("reason") if isinstance(select, dict) else "追踪目标选择失败"
                rospy.logwarn("memory select failed: %s", reason)
                self._say_key("target_lock_failed", "我看到了目标，但没有稳定锁定")
                task_reason = "看到了目标但没有稳定锁定"
                return

            self._publish_state("wait_tracker_lock")
            if not self._wait_tracker_lock():
                self._say_key("tracker_unstable", "目标追踪没有稳定下来")
                task_reason = "目标追踪没有稳定下来"
                return

            self._publish_state("auto_aim")
            if not self._auto_aim():
                self._say_key("autoaim_failed", "我没有对准目标")
                task_reason = "没有对准目标"
                return

            if self.stop_after_auto_aim:
                self._publish_state("auto_aim_done")
                self._say_key("autoaim_done", "已对准目标")
                task_result = "success"
                task_reason = "已对准目标后停止"
                return

            self._publish_state("grasp")
            if not self._call_trigger(self.grasp_service, "抓取"):
                self._say_key("grasp_failed", "抓取失败")
                task_reason = "抓取失败"
                return
            self._held_object = target_name or str(understanding.get("target_name") or "").strip()

            retract_ok = self._call_trigger(self.retract_service, "收回", required=False)
            rospy.loginfo("retract service result: %s", retract_ok)

            self._publish_state("return_to_person")
            self._say_key("returning", "我拿到了，现在回来")
            if not self._navigate_back_to_person(person_pose):
                self._say_key("return_failed", "我没有成功回到你的位置")
                task_reason = "抓到目标后没有成功回到说话人位置"
                return

            # Keep holding the object after returning. Release is intentionally
            # left to a separate/manual command so the object is not dropped.
            self._publish_state("returned_to_person")
            self._say_key("returned_to_person", "我回来了")
            task_result = "success"
            task_reason = "取物并返回说话人位置"

        except Exception as exc:
            rospy.logerr("fetch flow failed: %s", exc)
            self._say_key("task_error", "任务执行出错")
            task_reason = "任务执行异常：%s" % exc
        finally:
            self._remember_task(command, understanding, task_result, task_reason)
            self._stop_base()
            self._set_align(False)
            self._busy = False
            self._publish_state("idle")

    def _has_action_plan(self, understanding):
        plan = understanding.get("plan") if isinstance(understanding, dict) else None
        if not isinstance(plan, list):
            return False
        for step in plan:
            if isinstance(step, dict) and str(step.get("action") or "").strip():
                return True
        return False

    def _run_plan_pending(self, understanding):
        plan = understanding.get("plan") or []
        self._publish_state("plan_pending")
        rospy.loginfo(
            "complex action plan pending execution: %s",
            json.dumps(plan, ensure_ascii=False, sort_keys=True),
        )
        summary = self._summarize_action_plan(plan)
        if summary:
            rospy.loginfo("complex action plan summary: %s", summary)
        self._say("这个任务我已经拆成计划了，但当前还不能自动执行复杂多段任务")
        return False

    def _run_action_plan(self, command, understanding):
        plan = understanding.get("plan") or []
        self._publish_state("plan_execute")
        rospy.loginfo(
            "executing action plan: %s",
            json.dumps(plan, ensure_ascii=False, sort_keys=True),
        )
        summary = self._summarize_action_plan(plan)
        if summary:
            rospy.loginfo("action plan summary: %s", summary)

        if self._plan_needs_arm(plan) and not self._prepare_arm_before_task():
            self._say("机械臂没有回到初始状态，任务取消")
            return False

        ctx = {
            "command": command,
            "understanding": understanding,
            "found": {},
            "held_object": None,
            "person_pose": None,
            "placement_ready": False,
        }
        for index, step in enumerate(plan, start=1):
            if not isinstance(step, dict):
                continue
            action = str(step.get("action") or "").strip()
            self._publish_state("plan_step_%02d_%s" % (index, action or "unknown"))
            rospy.loginfo(
                "action plan step %d/%d: %s",
                index,
                len(plan),
                json.dumps(step, ensure_ascii=False, sort_keys=True),
            )
            ok = self._execute_plan_step(command, understanding, step, ctx)
            if not ok:
                rospy.logwarn("action plan failed at step %d: %s", index, action)
                self._set_align(False)
                self._stop_base()
                return False

        self._publish_state("plan_done")
        self._say("计划执行完成")
        return True

    @staticmethod
    def _plan_needs_arm(plan):
        arm_actions = {"grasp_object", "place_relative", "release_object"}
        for step in plan or []:
            if isinstance(step, dict) and str(step.get("action") or "").strip() in arm_actions:
                return True
        return False

    def _execute_plan_step(self, command, understanding, step, ctx):
        action = str(step.get("action") or "").strip()
        if action == "record_speaker":
            return self._execute_plan_record_speaker(ctx)
        if action == "navigate_to":
            location = str(step.get("location") or "").strip()
            if not location:
                self._say("计划里缺少导航地点")
                return False
            self._say("我去%s" % location, allow_llm=False)
            return self._navigate_to_location(location)
        if action == "find_object":
            return self._execute_plan_find_object(command, understanding, step, ctx)
        if action == "grasp_object":
            return self._execute_plan_grasp_object(command, understanding, step, ctx)
        if action == "place_relative":
            return self._execute_plan_place_relative(command, understanding, step, ctx)
        if action == "release_object":
            return self._execute_plan_release_object(ctx)
        if action == "return_to_speaker":
            return self._execute_plan_return_to_speaker(ctx)
        if action == "say":
            text = str(step.get("target") or step.get("text") or "").strip()
            if text:
                self._say(text)
            return True
        if action in ("ask_user", "chat"):
            self._say("这个计划步骤需要和你确认，我先停一下")
            return False
        self._say("计划里有我暂时不支持的动作")
        return False

    def _execute_plan_record_speaker(self, ctx):
        self._say_key("received_command", "收到，我先确认你的位置")
        person_pose = self._record_speaker_pose_after_stable_doa("plan speaker")
        if person_pose is None:
            self._say_key("no_person_pose", "我没有记录到你的位置，任务取消")
            return False
        ctx["person_pose"] = person_pose
        self.person_pose_pub.publish(person_pose)
        return True

    def _execute_plan_find_object(self, command, understanding, step, ctx):
        target = str(step.get("target") or "").strip()
        location = str(step.get("location") or "").strip()
        if not target:
            self._say("计划里缺少要找的物品")
            return False
        if location:
            self._say("我去%s找%s" % (location, target), allow_llm=False)
        else:
            self._say("我去找%s" % target, allow_llm=False)
        step_understanding = self._plan_step_understanding(understanding, step, target)
        semantic_hint = _semantic_hint_for_target(target)
        self._publish_state("plan_query_memory")
        memory = self._query_memory_for_target(
            command,
            step_understanding,
            target,
            semantic_hint,
            source_location=location,
        )
        fallback_target_obj = None
        if not self._memory_has_target(memory):
            memory = self._fallback_memory_from_likely_locations(
                command,
                step_understanding,
                target_name=target,
                semantic_hint=semantic_hint,
                preferred_location=location,
            )
            if not memory:
                self._say("我没有找到%s" % target)
                return False
            fallback_target_obj = memory.get("target_obj")

        point_id = str(memory.get("point_id") or "").strip()
        if not point_id:
            self._say("目标记忆缺少点位，不能导航")
            return False
        self._publish_state("plan_navigate_to_object")
        if not self._navigate_to_point(point_id):
            self._say("我没有到达%s所在位置" % target)
            return False

        found = {
            "target_id": str(memory.get("target_id") or "").strip(),
            "target_name": str(memory.get("target_name") or target).strip(),
            "point_id": point_id,
            "target_obj": fallback_target_obj or memory.get("target_obj"),
            "semantic_hint": semantic_hint,
        }
        self._plan_store_found_target(ctx, target, found)
        ctx["last_found"] = found
        return True

    def _execute_plan_grasp_object(self, command, understanding, step, ctx):
        target = str(step.get("target") or "").strip()
        found = self._plan_found_target(ctx, target)
        if not found:
            if not self._execute_plan_find_object(command, understanding, step, ctx):
                return False
            found = self._plan_found_target(ctx, target)
        if not found:
            self._say("我还没有找到要抓取的目标")
            return False
        target_id = str(found.get("target_id") or "").strip()
        target_name = str(found.get("target_name") or target).strip()
        if not target_id:
            self._say("目标记忆不完整，不能抓取")
            return False
        self._say("我开始抓%s" % target_name, allow_llm=False)
        if not self._confirm_select_and_align(
            command,
            target_id,
            target_name,
            "plan_grasp",
            target_obj=found.get("target_obj"),
        ):
            self._say("我没有稳定对准%s" % target_name)
            return False
        self._publish_state("plan_grasp")
        if not self._call_trigger(self.grasp_service, "抓取"):
            self._say("抓取失败")
            return False
        self._call_trigger(self.retract_service, "收回", required=False)
        ctx["held_object"] = found
        ctx["placement_ready"] = False
        self._say("我抓到了%s" % target_name, allow_llm=False)
        return True

    def _execute_plan_place_relative(self, command, understanding, step, ctx):
        reference = str(step.get("reference") or "").strip()
        location = str(step.get("location") or "").strip()
        side = str(step.get("side") or "").strip().lower()
        if side not in ("left", "right", "front", "back"):
            self._say("计划里的放置方向不完整")
            return False
        if not reference:
            self._say("计划里缺少放置参照物")
            return False
        if not ctx.get("held_object"):
            self._say("我还没有抓住要放置的物品")
            return False

        held_name = str((ctx.get("held_object") or {}).get("target_name") or step.get("target") or "物品").strip()
        self._say(
            "我去找%s，准备把%s放到%s" % (
                reference,
                held_name,
                _placement_side_text(side),
            ),
            allow_llm=False,
        )
        cached_reference = self._plan_found_target(ctx, reference, allow_last=False)
        if cached_reference:
            reference_id = str(cached_reference.get("target_id") or "").strip()
            reference_target_name = str(
                cached_reference.get("target_name") or reference
            ).strip()
            reference_point_id = str(cached_reference.get("point_id") or "").strip()
            reference_target_obj = cached_reference.get("target_obj")
            rospy.loginfo(
                "plan place_relative reuse cached reference: reference=%s target_id=%s point_id=%s",
                reference,
                reference_id,
                reference_point_id,
            )
        else:
            step_understanding = self._plan_step_understanding(understanding, step, reference)
            reference_memory = self._query_memory_for_target(
                command,
                step_understanding,
                reference,
                _reference_semantic_hint(reference),
                source_location=location,
            )
            reference_fallback_target_obj = None
            if not self._memory_has_target(reference_memory):
                reference_memory = self._fallback_memory_from_likely_locations(
                    command,
                    step_understanding,
                    target_name=reference,
                    semantic_hint=_reference_semantic_hint(reference),
                    preferred_location=location,
                )
                if not reference_memory:
                    self._say("我没有找到放置参照物")
                    return False
                reference_fallback_target_obj = reference_memory.get("target_obj")

            reference_id = str(reference_memory.get("target_id") or "").strip()
            reference_target_name = str(reference_memory.get("target_name") or reference).strip()
            reference_point_id = str(reference_memory.get("point_id") or "").strip()
            reference_target_obj = reference_fallback_target_obj or reference_memory.get("target_obj")
            self._plan_store_found_target(
                ctx,
                reference,
                {
                    "target_id": reference_id,
                    "target_name": reference_target_name,
                    "point_id": reference_point_id,
                    "target_obj": reference_target_obj,
                    "semantic_hint": _reference_semantic_hint(reference),
                },
            )
        if not reference_id or not reference_point_id:
            self._say("参照物记忆不完整，不能放置")
            return False
        if not self._navigate_to_point(reference_point_id):
            self._say("我没有到达参照物位置")
            return False
        if not self._call_trigger(self.placement_pre_align_service, "放置搜索姿态", required=True):
            self._say("我没有切换到放置搜索姿态")
            return False
        self._wait_placement_pre_align_settle()
        target_pixel_x = self._placement_reference_pixel_x(side)
        if not self._confirm_select_and_align(
            command,
            reference_id,
            reference_target_name,
            "plan_place_reference",
            target_pixel_x=target_pixel_x,
            tol_pixel_fine=self.placement_tol_pixel_fine,
            auto_aim_params=self.auto_aim_place_params,
            target_obj=reference_target_obj,
        ):
            self._say("我没有稳定对准放置参照物")
            return False
        if not self._call_trigger(self.retract_service, "放置前收回", required=True):
            self._say("我对准了参照物，但没有切回放置姿态")
            return False
        ctx["placement_ready"] = True
        self._say("我已对准放置参照物", allow_llm=False)
        return True

    def _execute_plan_release_object(self, ctx):
        if not ctx.get("held_object"):
            self._say("我没有夹着要释放的物品")
            return False
        held_name = str((ctx.get("held_object") or {}).get("target_name") or "物品").strip()
        self._say("我开始放下%s" % held_name, allow_llm=False)
        self._publish_state("plan_release")
        if not self._call_trigger(self.release_service, "释放", required=True):
            self._say("释放失败")
            return False
        self._call_trigger(self.retract_service, "释放后收回", required=False)
        ctx["held_object"] = None
        ctx["placement_ready"] = False
        self._say("我已经放好了", allow_llm=False)
        return True

    def _execute_plan_return_to_speaker(self, ctx):
        person_pose = ctx.get("person_pose")
        if person_pose is None:
            self._say("计划里没有记录说话人位置")
            return False
        return self._navigate_back_to_person(person_pose)

    def _plan_step_understanding(self, understanding, step, target):
        payload = dict(understanding or {})
        payload["target_name"] = str(target or "").strip()
        payload["semantic_hint"] = _semantic_hint_for_target(target)
        location = str(step.get("location") or "").strip()
        if location:
            payload["source_location"] = location
        search_locations = step.get("search_locations")
        if isinstance(search_locations, list) and search_locations:
            payload["search_locations"] = search_locations
        elif location:
            payload["search_locations"] = [location]
        return payload

    @staticmethod
    def _plan_store_found_target(ctx, target, found):
        found_map = ctx.setdefault("found", {})
        keys = []
        for value in (
            target,
            (found or {}).get("target_name"),
            ((found or {}).get("target_obj") or {}).get("main_name")
            if isinstance((found or {}).get("target_obj"), dict)
            else "",
        ):
            text = str(value or "").strip()
            if text and text not in keys:
                keys.append(text)
        for key in keys:
            found_map[key] = found

    @staticmethod
    def _plan_found_target(ctx, target, allow_last=True):
        found = ctx.get("found") or {}
        if target and target in found:
            return found[target]
        return ctx.get("last_found") if allow_last else None

    def _summarize_action_plan(self, plan):
        parts = []
        action_names = {
            "record_speaker": "记录说话人",
            "navigate_to": "导航",
            "find_object": "找物品",
            "grasp_object": "抓取",
            "place_relative": "相对放置",
            "release_object": "释放",
            "return_to_speaker": "返回给你",
            "say": "播报",
            "ask_user": "询问",
            "chat": "闲聊",
        }
        for index, step in enumerate(plan or [], start=1):
            if not isinstance(step, dict):
                continue
            action = str(step.get("action") or "").strip()
            name = action_names.get(action, action)
            target = str(step.get("target") or "").strip()
            location = str(step.get("location") or "").strip()
            reference = str(step.get("reference") or "").strip()
            side = str(step.get("side") or "").strip()
            detail = target or location or reference
            if action == "place_relative" and reference:
                detail = "%s -> %s %s" % (target or "当前物品", reference, side or "")
            if detail:
                parts.append("%d.%s(%s)" % (index, name, detail))
            else:
                parts.append("%d.%s" % (index, name))
        return " | ".join(parts)

    def _run_navigation_task(self, understanding):
        location = str(understanding.get("destination_location") or "").strip()
        if not location:
            for task in understanding.get("tasks") or []:
                if isinstance(task, dict) and str(task.get("action") or "") == "navigate":
                    location = str(task.get("location") or "").strip()
                    break
        if not location:
            self._say("我没有识别到要去的位置")
            return False
        self._publish_state("navigate_to_location")
        ok = self._navigate_to_location(location)
        if ok:
            self._say("我到了")
        else:
            self._say("我没有到达指定位置")
        return ok

    def _run_task_chain(self, understanding):
        tasks = understanding.get("tasks") or []
        nav_tasks = [
            str(task.get("location") or "").strip()
            for task in tasks
            if isinstance(task, dict) and str(task.get("action") or "") == "navigate"
        ]
        nav_tasks = [location for location in nav_tasks if location]
        if not nav_tasks:
            self._say("我没有识别到任务链里的位置")
            return False
        for index, location in enumerate(nav_tasks, start=1):
            self._publish_state("task_chain_navigate_%d" % index)
            rospy.loginfo("task chain step %d/%d navigate to %s", index, len(nav_tasks), location)
            if not self._navigate_to_location(location):
                self._say("我没有完成第%d个位置" % index)
                return False
        self._say("任务链完成")
        return True

    def _run_transfer_task(self, command, understanding):
        source_location = str(understanding.get("source_location") or "").strip()
        destination_location = str(understanding.get("destination_location") or "").strip()
        target_name = str(understanding.get("target_name") or "").strip()
        release_at_destination = _as_bool(understanding.get("release_at_destination", False))

        for task in understanding.get("tasks") or []:
            if not isinstance(task, dict):
                continue
            action = str(task.get("action") or "").strip()
            if action == "navigate" and not source_location:
                source_location = str(task.get("location") or "").strip()
            elif action == "navigate":
                destination_location = str(task.get("location") or "").strip() or destination_location
            elif action == "detect_and_grasp" and not target_name:
                target_name = str(task.get("target_name") or "").strip()

        if not source_location or not destination_location or not target_name:
            rospy.logwarn(
                "transfer task incomplete: source=%s destination=%s target=%s payload=%s",
                source_location,
                destination_location,
                target_name,
                json.dumps(understanding, ensure_ascii=False, sort_keys=True),
            )
            self._say("搬运任务信息不完整")
            return False

        self._publish_state("transfer_navigate_to_source")
        if not self._navigate_to_location(source_location):
            self._say("我没有到达物品所在位置")
            return False

        self._publish_state("transfer_query_memory")
        memory = self._query_memory(command, understanding)
        fallback_target_obj = None
        if not memory or not memory.get("success"):
            reason = memory.get("reason") if isinstance(memory, dict) else "记忆查询失败"
            rospy.logwarn("transfer memory query failed: %s", reason)
            memory = self._fallback_memory_at_location(command, understanding, source_location)
            if not memory:
                self._say("我没有找到这个物品的记忆")
                return False
            fallback_target_obj = memory.get("target_obj")
        target_id = str(memory.get("target_id") or "").strip()
        memory_target_name = str(memory.get("target_name") or target_name).strip()
        if not target_id:
            self._say("目标记忆不完整，不能识别抓取")
            return False

        self._publish_state("transfer_confirm_target")
        confirm = self._confirm_target(
            target_id,
            memory_target_name,
            command,
            target_obj=fallback_target_obj,
        )
        rospy.loginfo(
            "transfer target confirm result: %s",
            json.dumps(confirm or {}, ensure_ascii=False, sort_keys=True),
        )
        if not confirm or not confirm.get("target_found"):
            self._say("我到了源位置，但没有确认到目标")
            return False

        self._publish_state("transfer_select_tracking_target")
        select = self._select_tracking_target(
            target_id,
            memory_target_name,
            command,
            target_obj=fallback_target_obj,
        )
        rospy.loginfo(
            "transfer tracking target select result: %s",
            json.dumps(select or {}, ensure_ascii=False, sort_keys=True),
        )
        if not select or not select.get("success"):
            self._say("我看到了目标，但没有稳定锁定")
            return False

        self._publish_state("transfer_wait_tracker_lock")
        if not self._wait_tracker_lock():
            self._say("目标追踪没有稳定下来")
            return False

        self._publish_state("transfer_auto_aim")
        if not self._auto_aim():
            self._say("我没有对准目标")
            return False

        self._publish_state("transfer_grasp")
        if not self._call_trigger(self.grasp_service, "抓取"):
            self._say("抓取失败")
            return False
        self._held_object = memory_target_name or target_name
        self._call_trigger(self.retract_service, "收回", required=False)

        self._publish_state("transfer_navigate_to_destination")
        if not self._navigate_to_location(destination_location):
            self._say("我抓到了物品，但没有到达目标位置")
            return False

        if release_at_destination:
            self._publish_state("transfer_release")
            if not self._call_trigger(self.release_service, "释放", required=False):
                self._say("我到了目标位置，但释放失败")
                return False
            self._held_object = ""
            self._say("我已送到并放下")
        else:
            self._say("我已送到目标位置，并保持夹持")
        return True

    def _run_deliver_to_person_task(self, command, understanding):
        source_location = str(understanding.get("source_location") or "").strip()
        target_name = str(understanding.get("target_name") or "").strip()
        person_desc = str(understanding.get("delivery_target") or "").strip()
        if not source_location or not target_name or not person_desc:
            rospy.logwarn(
                "deliver_to_person incomplete: source=%s target=%s person=%s payload=%s",
                source_location,
                target_name,
                person_desc,
                json.dumps(understanding, ensure_ascii=False, sort_keys=True),
            )
            self._say("人物交付任务信息不完整")
            return False

        self._publish_state("deliver_query_object_memory")
        memory = self._query_memory(command, understanding)
        fallback_target_obj = None
        if not memory or not memory.get("success"):
            reason = memory.get("reason") if isinstance(memory, dict) else "记忆查询失败"
            rospy.logwarn("deliver object memory query failed: %s", reason)
            memory = self._fallback_memory_at_location(command, understanding, source_location)
            if not memory:
                self._say("我没有找到这个物品的记忆")
                return False
            fallback_target_obj = memory.get("target_obj")

        target_id = str(memory.get("target_id") or "").strip()
        memory_target_name = str(memory.get("target_name") or target_name).strip()
        point_id = str(memory.get("point_id") or "").strip()
        if not target_id or not point_id:
            self._say("目标记忆不完整，不能识别抓取")
            return False

        self._publish_state("deliver_navigate_to_object")
        if not self._navigate_to_point(point_id):
            self._say("我没有到达物品所在位置")
            return False

        if not self._confirm_select_and_align(
            "抓取%s" % memory_target_name,
            target_id,
            memory_target_name,
            "deliver_object",
            target_obj=fallback_target_obj,
        ):
            self._say("我没有稳定抓到要送的物品")
            return False

        self._publish_state("deliver_grasp")
        if not self._call_trigger(self.grasp_service, "抓取"):
            self._say("抓取失败")
            return False
        self._held_object = memory_target_name or target_name
        self._call_trigger(self.retract_service, "收回", required=False)

        self._say("我拿到了，现在找%s" % person_desc, allow_llm=False)
        person_pose = self._search_delivery_person(person_desc)
        if person_pose is None:
            self._say("我没有找到%s" % person_desc, allow_llm=False)
            return False

        self.person_pose_pub.publish(person_pose)
        self._publish_state("deliver_navigate_to_person")
        if not self._navigate_back_to_person(person_pose):
            self._say("我找到了人，但没有成功导航过去")
            return False

        self._publish_state("deliver_arrived_person")
        self._say("我到了", allow_llm=False)
        return True

    def _is_relative_placement_task(self, understanding):
        if not isinstance(understanding, dict):
            return False
        reference = str(understanding.get("placement_reference") or "").strip()
        side = str(understanding.get("placement_side") or "").strip().lower()
        return bool(reference and side in ("left", "right", "front", "back"))

    def _run_relative_placement_task(self, command, understanding):
        target_name = str(understanding.get("target_name") or "").strip()
        reference_name = str(understanding.get("placement_reference") or "").strip()
        side = str(understanding.get("placement_side") or "").strip().lower()

        if not target_name or not reference_name or side not in ("left", "right", "front", "back"):
            rospy.logwarn(
                "relative placement task incomplete: target=%s reference=%s side=%s payload=%s",
                target_name,
                reference_name,
                side,
                json.dumps(understanding, ensure_ascii=False, sort_keys=True),
            )
            self._say("放置任务信息不完整")
            return False

        self._publish_state("placement_query_object_memory")
        object_memory = self._query_memory_for_target(
            command,
            understanding,
            target_name,
            str(understanding.get("semantic_hint") or "").strip(),
            source_location=str(understanding.get("source_location") or "").strip(),
        )
        object_fallback_target_obj = None
        if not self._memory_has_target(object_memory):
            object_memory = self._fallback_memory_from_likely_locations(
                command,
                understanding,
                target_name=target_name,
                semantic_hint=str(understanding.get("semantic_hint") or "").strip(),
                preferred_location=str(understanding.get("source_location") or "").strip(),
            )
            if not object_memory:
                self._say("我没有找到要搬的物品")
                return False
            object_fallback_target_obj = object_memory.get("target_obj")

        object_id = str(object_memory.get("target_id") or "").strip()
        object_name = str(object_memory.get("target_name") or target_name).strip()
        object_point_id = str(object_memory.get("point_id") or "").strip()

        self._say("我先去找%s" % object_name)
        self._publish_state("placement_navigate_to_object")
        if not self._navigate_to_point(object_point_id):
            self._say("我没有到达要搬的物品位置")
            return False

        if not self._confirm_select_and_align(
            "抓取%s" % object_name,
            object_id,
            object_name,
            "placement_object",
            target_obj=object_fallback_target_obj,
        ):
            self._say("我没有稳定抓到要搬的物品")
            return False

        self._publish_state("placement_grasp")
        if not self._call_trigger(self.grasp_service, "抓取"):
            self._say("抓取失败")
            return False
        self._held_object = object_name or target_name
        self._call_trigger(self.retract_service, "收回", required=False)

        self._publish_state("placement_query_reference_memory")
        reference_memory = self._query_memory_for_target(
            command,
            understanding,
            reference_name,
            _reference_semantic_hint(reference_name),
            source_location=str(understanding.get("destination_location") or "").strip(),
        )
        reference_fallback_target_obj = None
        if not self._memory_has_target(reference_memory):
            reference_memory = self._fallback_memory_from_likely_locations(
                command,
                understanding,
                target_name=reference_name,
                semantic_hint=_reference_semantic_hint(reference_name),
                preferred_location=str(understanding.get("destination_location") or "").strip(),
            )
            if not reference_memory:
                self._say("我没有找到放置参照物")
                return False
            reference_fallback_target_obj = reference_memory.get("target_obj")

        reference_id = str(reference_memory.get("target_id") or "").strip()
        reference_target_name = str(reference_memory.get("target_name") or reference_name).strip()
        reference_point_id = str(reference_memory.get("point_id") or "").strip()

        self._say("我去找%s，准备放到%s" % (reference_target_name, _placement_side_text(side)))
        self._publish_state("placement_navigate_to_reference")
        if not self._navigate_to_point(reference_point_id):
            self._say("我没有到达参照物位置")
            return False

        self._publish_state("placement_pre_align_arm_pose")
        if not self._call_trigger(self.placement_pre_align_service, "放置搜索姿态", required=True):
            self._say("我没有切换到放置搜索姿态")
            return False
        self._wait_placement_pre_align_settle()

        target_pixel_x = self._placement_reference_pixel_x(side)
        rospy.loginfo(
            "relative placement align: side=%s reference=%s target_pixel_x=%d",
            side,
            reference_target_name,
            target_pixel_x,
        )
        if not self._confirm_select_and_align(
            "确认%s作为放置参照物" % reference_target_name,
            reference_id,
            reference_target_name,
            "placement_reference",
            target_pixel_x=target_pixel_x,
            tol_pixel_fine=self.placement_tol_pixel_fine,
            auto_aim_params=self.auto_aim_place_params,
            target_obj=reference_fallback_target_obj,
        ):
            self._say("我没有稳定对准放置参照物")
            return False

        self._publish_state("placement_restore_arm_pose")
        if not self._call_trigger(self.retract_service, "放置前收回", required=True):
            self._say("我对准了参照物，但没有切回放置姿态")
            return False

        self._publish_state("placement_release")
        if not self._call_trigger(self.release_service, "释放", required=True):
            self._say("我对准了参照物，但释放失败")
            return False
        self._held_object = ""

        self._publish_state("placement_retract_after_release")
        self._call_trigger(self.retract_service, "释放后收回", required=False)
        self._say("我已经放好了")
        return True

    def _confirm_select_and_align(
        self,
        command,
        target_id,
        target_name,
        state_prefix,
        target_pixel_x=None,
        tol_pixel_fine=None,
        auto_aim_params=None,
        target_obj=None,
    ):
        self._publish_state("%s_confirm_target" % state_prefix)
        confirm = self._confirm_target(target_id, target_name, command, target_obj=target_obj)
        rospy.loginfo(
            "%s confirm result: %s",
            state_prefix,
            json.dumps(confirm or {}, ensure_ascii=False, sort_keys=True),
        )
        if not confirm or not confirm.get("target_found"):
            return False

        self._publish_state("%s_select_tracking_target" % state_prefix)
        select = self._select_tracking_target(
            target_id,
            target_name,
            command,
            target_obj=target_obj,
        )
        rospy.loginfo(
            "%s tracking select result: %s",
            state_prefix,
            json.dumps(select or {}, ensure_ascii=False, sort_keys=True),
        )
        if not select or not select.get("success"):
            return False

        self._publish_state("%s_wait_tracker_lock" % state_prefix)
        if not self._wait_tracker_lock():
            return False

        self._publish_state("%s_auto_aim" % state_prefix)
        return self._auto_aim(
            target_pixel_x=target_pixel_x,
            tol_pixel_fine=tol_pixel_fine,
            auto_aim_params=auto_aim_params,
        )

    def _query_memory_for_target(self, command, understanding, target_name, semantic_hint="", source_location=None):
        payload = dict(understanding or {})
        payload["target_name"] = str(target_name or "").strip()
        payload["semantic_hint"] = str(semantic_hint or "").strip()
        if source_location is not None:
            payload["source_location"] = str(source_location or "").strip()
        query_command = "寻找%s" % payload["target_name"]
        return self._query_memory(query_command, payload)

    def _memory_has_target(self, memory):
        if not memory or not memory.get("success"):
            reason = memory.get("reason") if isinstance(memory, dict) else "记忆查询失败"
            rospy.logwarn("memory query failed in relative placement: %s", reason)
            return False
        return bool(str(memory.get("target_id") or "").strip() and str(memory.get("point_id") or "").strip())

    def _is_memory_location_question(self, command, understanding):
        target_name = str((understanding or {}).get("target_name") or "").strip()
        if not target_name:
            return False
        text = str(command or "")
        return (
            any(key in text for key in ("在哪", "哪里", "什么位置", "位置"))
            or (any(key in text for key in ("记得", "知道")) and "在" in text)
        )

    def _answer_memory_location_question(self, command, understanding):
        target_name = str((understanding or {}).get("target_name") or "").strip()
        if not target_name:
            self._say("你想查哪个物品的位置")
            return False
        self._publish_state("query_memory_location")
        memory = self._query_memory(command, understanding)
        if not memory or not memory.get("success"):
            self._say("我没有在记忆里查到%s的位置" % target_name)
            return False

        point_id = str(memory.get("point_id") or "").strip()
        point = self._patrol_points.get(point_id, {})
        location = str(point.get("name") or point_id or "记忆点").strip()
        target_obj = memory.get("target_obj") or {}
        remembered_name = str(
            target_obj.get("main_name")
            or memory.get("target_name")
            or target_name
        ).strip()

        if self._memory_name_exact_enough(target_name, remembered_name):
            self._say("我记得%s在%s" % (target_name, location))
        else:
            self._say(
                "记忆里没有完全匹配%s，但有一个%s，可能在%s"
                % (target_name, remembered_name or "相似物品", location)
            )
        return True

    def _memory_name_exact_enough(self, target_name, remembered_name):
        target = "".join(str(target_name or "").lower().split())
        remembered = "".join(str(remembered_name or "").lower().split())
        return bool(target and remembered and (target in remembered or remembered in target))

    def _run_come_to_speaker_task(self):
        self._publish_state("come_to_speaker_turn")
        person_pose = self._record_speaker_pose_after_stable_doa("come_to_speaker")
        if person_pose is None:
            self._say_key("no_person_pose", "我没有记录到你的位置，任务取消")
            return False

        self.person_pose_pub.publish(person_pose)
        rospy.loginfo(
            "come_to_speaker person pose: frame=%s x=%.3f y=%.3f z=%.3f",
            person_pose.header.frame_id,
            person_pose.pose.position.x,
            person_pose.pose.position.y,
            person_pose.pose.position.z,
        )
        self._publish_state("come_to_speaker_navigate")
        if not self._navigate_back_to_person(person_pose):
            self._say_key("return_failed", "我没有成功到你的位置")
            return False
        self._publish_state("come_to_speaker_arrived")
        self._say("我到了", allow_llm=False)
        return True

    def _fallback_memory_at_location(self, command, understanding, location):
        return self._fallback_memory_from_likely_locations(
            command,
            understanding,
            preferred_location=location,
        )

    def _fallback_memory_from_likely_locations(
        self,
        command,
        understanding,
        target_name=None,
        semantic_hint=None,
        preferred_location=None,
    ):
        target_name = str(
            target_name
            if target_name is not None
            else (understanding or {}).get("target_name") or ""
        ).strip()
        semantic_hint = str(
            semantic_hint
            if semantic_hint is not None
            else (understanding or {}).get("semantic_hint") or ""
        ).strip()
        if not target_name:
            return None

        candidates = self._likely_point_ids_for_target(
            target_name,
            semantic_hint,
            preferred_location
            if preferred_location is not None
            else str((understanding or {}).get("source_location") or "").strip(),
            understanding=understanding,
        )
        if not candidates:
            rospy.logwarn("no fallback locations for target=%s", target_name)
            return None

        for point_id in candidates:
            target_obj = self._temporary_target_obj(target_name, semantic_hint, point_id)
            if not target_obj:
                rospy.logwarn("cannot build fallback target object for %s", target_name)
                return None
            point_name = str(self._patrol_points.get(point_id, {}).get("name") or point_id)
            rospy.loginfo(
                "memory fallback candidate: target=%s semantic=%s point=%s(%s)",
                target_name,
                semantic_hint,
                point_id,
                point_name,
            )
            self._say("记忆里没找到%s，我去%s现场找找" % (target_name, point_name))
            if not self._navigate_to_point(point_id):
                rospy.logwarn("fallback navigate failed: point=%s target=%s", point_id, target_name)
                continue
            self._publish_state("fallback_pre_align_arm_pose")
            if not self._call_trigger(self.placement_pre_align_service, "现场搜索姿态", required=True):
                rospy.logwarn("fallback pre-align arm pose failed: point=%s target=%s", point_id, target_name)
                return None
            self._publish_state("fallback_confirm_target")
            confirm = self._confirm_target(
                target_obj["id"],
                target_name,
                command,
                target_obj=target_obj,
            )
            rospy.loginfo(
                "fallback confirm result: %s",
                json.dumps(confirm or {}, ensure_ascii=False, sort_keys=True),
            )
            if confirm and confirm.get("target_found"):
                return {
                    "success": True,
                    "target_id": target_obj["id"],
                    "target_name": target_name,
                    "point_id": point_id,
                    "target_obj": target_obj,
                    "selection_method": "location_fallback",
                }
        return None

    def _likely_point_ids_for_target(
        self,
        target_name,
        semantic_hint,
        preferred_location="",
        understanding=None,
    ):
        point_ids = []
        preferred_location = str(preferred_location or "").strip()
        if preferred_location:
            point_id, _ = self._resolve_point(preferred_location)
            if point_id:
                point_ids.append(point_id)

        for name in self._search_locations_from_understanding(understanding):
            point_id, _ = self._resolve_point(name)
            if point_id and point_id not in point_ids:
                point_ids.append(point_id)

        if not point_ids:
            rospy.logwarn(
                "LLM did not provide usable search_locations for target=%s semantic=%s; scan all configured points",
                target_name,
                semantic_hint,
            )
            for point_id, point in self._patrol_points.items():
                if _as_bool(point.get("scan", True)) and point_id not in point_ids:
                    point_ids.append(point_id)
        return point_ids

    def _search_locations_from_understanding(self, understanding):
        raw = (understanding or {}).get("search_locations") or []
        if isinstance(raw, str):
            raw = [raw]
        if not isinstance(raw, (list, tuple)):
            return []
        result = []
        seen = set()
        for item in raw:
            name = str(item or "").strip()
            if not name or name in seen:
                continue
            seen.add(name)
            result.append(name)
        return result

    def _temporary_target_obj(self, target_name, semantic_hint, point_id):
        yolo_class, backups = _target_yolo_classes(target_name, semantic_hint)
        if not yolo_class:
            yolo_class = CUSTOM_YOLO_CLASSES[0]
            backups = CUSTOM_YOLO_CLASSES[1:]
        else:
            full_backups = list(backups or [])
            for class_name in CUSTOM_YOLO_CLASSES:
                if class_name != yolo_class and class_name not in full_backups:
                    full_backups.append(class_name)
            backups = full_backups
        safe_name = "".join(ch for ch in str(target_name) if ch.isalnum())[:20] or "target"
        return {
            "id": "tmp_%s_%s" % (safe_name, point_id),
            "main_name": target_name,
            "possible_names": _possible_names_for_target(target_name),
            "yolo_class": yolo_class,
            "backup_yolo_classes": backups,
            "point_id": point_id,
            "detected_at_point": point_id,
        }

    def _placement_reference_pixel_x(self, side):
        if side == "left":
            # Put the reference object on the right side of the image, leaving
            # free space on its left for the carried object.
            return self.placement_left_reference_pixel_x
        if side == "right":
            return self.placement_right_reference_pixel_x
        return self.auto_aim_center_pixel_x

    def _navigate_to_location(self, location):
        point_id, point = self._resolve_point(location)
        if not point:
            rospy.logwarn("location not found in patrol points: %s", location)
            self._say("我没有找到%s这个位置" % location)
            return False
        rospy.loginfo(
            "navigate location resolved: location=%s point_id=%s name=%s",
            location,
            point_id,
            point.get("name", ""),
        )
        return self._send_goal(point, self.nav_timeout_sec)

    def _resolve_point(self, location):
        normalized = self._normalize_location_name(location)
        for point_id, point in self._patrol_points.items():
            names = [point_id, str(point.get("name") or "")]
            aliases = point.get("aliases") or []
            if isinstance(aliases, list):
                names.extend(str(alias) for alias in aliases)
            if normalized in {self._normalize_location_name(name) for name in names if name}:
                return point_id, point
        return "", None

    def _normalize_location_name(self, value):
        text = str(value or "").strip().lower()
        return (
            text.replace("_", "")
            .replace("-", "")
            .replace(" ", "")
            .replace("号点", "")
        )

    def _turn_to_speaker(self):
        doa_deg = self._most_frequent_doa()
        if doa_deg is None:
            rospy.logwarn("not enough DOA samples; skip speaker turn")
            self._say_key("doa_unstable", "声源方向不稳定，我没有记录你的位置")
            return False
        rospy.loginfo("selected DOA: %.1f deg", doa_deg)
        self._publish_state("turn_to_speaker")
        return self._turn_by_relative_angle(doa_deg)

    def _record_speaker_pose_after_stable_doa(self, label):
        turn_ok = self._turn_to_speaker()
        rospy.loginfo("%s turn result: %s", label, turn_ok)
        if not turn_ok:
            rospy.logwarn(
                "%s DOA unstable or turn failed; discard any current person pose",
                label,
            )
            return None

        person_pose = self._record_person_pose(allow_robot_fallback=False)
        if person_pose is None and self.person_retry_enabled:
            rospy.logwarn("%s person pose unavailable after stable DOA; retry DOA", label)
            person_pose = self._retry_record_person_pose()
        return person_pose

    def _retry_record_person_pose(self):
        self._publish_state("retry_find_person")
        self._say_key(
            "person_retry_prompt",
            self.person_retry_prompt,
        )
        self._wait_tts_idle(timeout_sec=8.0)
        got_doa = self._collect_retry_doa_samples()
        if not got_doa:
            rospy.logwarn("person retry did not collect enough DOA samples")
            self._say_key("doa_unstable", "声源方向不稳定，我没有记录你的位置")
            rospy.logwarn("discard person pose because retry DOA is unstable")
            return None
        turn_ok = self._turn_to_speaker()
        rospy.loginfo("speaker retry turn result: %s", turn_ok)
        if not turn_ok:
            rospy.logwarn("discard person pose because retry speaker turn failed")
            return None
        return self._record_person_pose(allow_robot_fallback=False)

    def _collect_retry_doa_samples(self):
        start = time.monotonic()
        deadline = time.monotonic() + self.person_retry_listen_timeout_sec
        min_deadline = start + min(
            max(self.person_retry_min_listen_sec, 0.0),
            max(self.person_retry_listen_timeout_sec, 0.0),
        )
        with self._doa_lock:
            self._doa_recording = True
            self._doa_samples.clear()
            self._latched_doa_samples = None
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with self._doa_lock:
                    sample_count = len(self._doa_samples)
                if sample_count >= self.min_doa_samples and time.monotonic() >= min_deadline:
                    return True
                rate.sleep()
            with self._doa_lock:
                return len(self._doa_samples) >= self.min_doa_samples
        finally:
            with self._doa_lock:
                self._doa_recording = False
                self._latched_doa_samples = list(self._doa_samples)
            rospy.loginfo(
                "person retry latched %d DOA samples",
                len(self._latched_doa_samples or []),
            )

    def _wait_tts_idle(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        start_grace_deadline = time.monotonic() + 1.0
        saw_playing = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._doa_lock:
                playing = self._tts_playing
            if playing:
                saw_playing = True
            elif saw_playing:
                return True
            elif time.monotonic() >= start_grace_deadline:
                return True
            time.sleep(0.05)
        with self._doa_lock:
            return not self._tts_playing

    def _fallback_person_pose_after_no_person(self):
        if not self.use_robot_pose_if_no_person:
            return None
        rospy.logwarn("person pose unavailable after retry; fallback to current robot pose")
        self._say_key("no_person_pose_fallback", "没有找到你，我先继续任务")
        pose = self._current_robot_pose()
        if pose is not None:
            rospy.logwarn(
                "fallback robot pose used as person pose: frame=%s x=%.3f y=%.3f",
                pose.header.frame_id,
                pose.pose.position.x,
                pose.pose.position.y,
            )
        return pose

    def _record_person_pose(self, allow_robot_fallback=True):
        self._publish_state("record_person_pose")
        pose = self._wait_person_pose_in_map()
        if pose is not None:
            rospy.loginfo(
                "recorded person pose from tf %s->%s: x=%.3f y=%.3f z=%.3f",
                self.map_frame,
                self.vision_frame,
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            return pose
        if not allow_robot_fallback:
            return None
        return self._fallback_person_pose_after_no_person()

    def _search_delivery_person(self, person_desc):
        person_desc = str(person_desc or "").strip() or "目标人物"
        if self.deliver_person_backup_m > 0.0:
            self._publish_state("deliver_person_backup")
            if not self._drive_linear_distance(-abs(self.deliver_person_backup_m), self.deliver_person_backup_speed):
                rospy.logwarn("deliver person backup did not complete cleanly; continue searching")

        max_deg = max(0.0, float(self.deliver_person_search_max_deg))
        step_deg = max(5.0, abs(float(self.deliver_person_search_step_deg)))
        turned = 0.0
        attempt = 0
        while not rospy.is_shutdown() and turned <= max_deg + 1e-6:
            attempt += 1
            self._publish_state("deliver_person_confirm")
            confirm = self._confirm_delivery_person(person_desc)
            rospy.loginfo(
                "deliver person confirm attempt=%d turned=%.1f result=%s",
                attempt,
                turned,
                json.dumps(confirm or {}, ensure_ascii=False, sort_keys=True),
            )
            if confirm and confirm.get("target_found"):
                pose = self._person_pose_from_confirm(confirm)
                if pose is None:
                    pose = self._record_person_pose(allow_robot_fallback=False)
                if pose is not None:
                    rospy.loginfo(
                        "deliver person pose selected: frame=%s x=%.3f y=%.3f z=%.3f",
                        pose.header.frame_id,
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                    )
                    return pose
                rospy.logwarn("deliver person VLM matched but no 3D pose was available")

            if turned >= max_deg:
                break
            rotate_deg = min(step_deg, max_deg - turned)
            if rotate_deg <= 0.0:
                break
            self._publish_state("deliver_person_rotate")
            if not self._rotate_relative_slow(rotate_deg):
                rospy.logwarn("deliver person search rotation failed at %.1fdeg", turned)
                break
            turned += rotate_deg
        return None

    def _confirm_delivery_person(self, person_desc):
        target_obj = {
            "id": "deliver_person",
            "main_name": person_desc,
            "yolo_class": "person",
            "backup_yolo_classes": [],
        }
        return self._confirm_target(
            "deliver_person",
            person_desc,
            "确认%s" % person_desc,
            target_obj=target_obj,
        )

    def _person_pose_from_confirm(self, confirm):
        if not isinstance(confirm, dict):
            return None
        point = confirm.get("object_3d_position")
        frame = str(confirm.get("object_3d_frame") or "").strip()
        if not isinstance(point, (list, tuple)) or len(point) < 3 or not frame:
            return None
        try:
            stamped = PointStamped()
            stamped.header.stamp = rospy.Time(0)
            stamped.header.frame_id = frame
            stamped.point.x = float(point[0])
            stamped.point.y = float(point[1])
            stamped.point.z = float(point[2])
            self.tf_listener.waitForTransform(
                self.map_frame,
                frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            mapped = self.tf_listener.transformPoint(self.map_frame, stamped)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = mapped.point.x
            pose.pose.position.y = mapped.point.y
            pose.pose.position.z = mapped.point.z
            try:
                robot_x, robot_y, _ = self._lookup_base_pose()
                yaw = math.atan2(mapped.point.y - robot_y, mapped.point.x - robot_x)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                yaw = 0.0
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            return pose
        except Exception as exc:
            rospy.logwarn("failed to transform confirmed person 3D point to map: %s", exc)
            return None

    def _drive_linear_distance(self, distance_m, speed_mps):
        if self.dry_run:
            rospy.loginfo("dry_run: skip linear drive %.2fm", distance_m)
            return True
        distance = float(distance_m)
        speed = max(0.03, abs(float(speed_mps)))
        direction = 1.0 if distance >= 0.0 else -1.0
        try:
            start_x, start_y, _ = self._lookup_base_pose()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("cannot lookup base pose for linear drive: %s", exc)
            return False

        target = abs(distance)
        deadline = time.monotonic() + max(2.0, target / speed + 2.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                x, y, _ = self._lookup_base_pose()
                traveled = math.hypot(x - start_x, y - start_y)
                if traveled >= target:
                    self._stop_base()
                    return True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            cmd = Twist()
            cmd.linear.x = direction * speed
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self._stop_base()
        return False

    def _rotate_relative_slow(self, angle_deg):
        if self.dry_run:
            rospy.loginfo("dry_run: skip slow rotate %.1fdeg", angle_deg)
            return True
        try:
            _, _, start_yaw = self._lookup_base_pose()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("cannot lookup base yaw for slow rotate: %s", exc)
            return False
        target_yaw = normalize_rad(start_yaw + math.radians(float(angle_deg)))
        angular_speed = max(0.05, abs(float(self.deliver_person_search_angular_speed)))
        deadline = time.monotonic() + max(4.0, abs(math.radians(float(angle_deg))) / angular_speed + 3.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                _, _, current_yaw = self._lookup_base_pose()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue
            err = normalize_rad(target_yaw - current_yaw)
            if abs(math.degrees(err)) <= self.turn_tolerance_deg:
                self._stop_base()
                return True
            cmd = Twist()
            cmd.angular.z = math.copysign(angular_speed, err) * self.turn_direction_sign
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self._stop_base()
        return False

    def _wait_placement_pre_align_settle(self):
        delay = max(0.0, float(self.placement_pre_align_settle_sec))
        if delay <= 0.0:
            return
        rospy.loginfo(
            "wait %.2fs for placement pre-align arm pose to settle before vision confirm",
            delay,
        )
        rospy.sleep(delay)

    def _understand_command(self, command):
        if not self.enable_task_understanding:
            return None
        request_id = str(int(time.time() * 1000))
        recent_context = self._build_recent_task_context()
        payload = {
            "request_id": request_id,
            "user_command": command,
            "query": command,
        }
        if recent_context:
            payload["recent_context"] = recent_context
        with self._understanding_condition:
            self._pending_understanding_request_id = request_id
            self._understanding_result = None
        self._publish_state("understand_command")
        self.task_understanding_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        result = self._wait_json_result(
            self._understanding_condition,
            lambda: self._understanding_result,
            self.task_understanding_timeout_sec,
        )
        if result is None:
            rospy.logwarn("task understanding timeout; fallback to raw command")
            return None
        if not result.get("success"):
            rospy.logwarn("task understanding returned fallback/failure: %s", result.get("reason", ""))
        return result

    def _load_task_memory(self):
        path = self.task_memory_path
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, list) else []
        except IOError:
            return []
        except Exception as exc:
            rospy.logwarn("failed to load task memory %s: %s", path, exc)
            return []

    def _load_last_held_object(self):
        records = self._load_task_memory()
        for item in reversed(records):
            if not isinstance(item, dict):
                continue
            return str(item.get("held_object_after") or "").strip()
        return ""

    def _save_task_memory(self, records):
        path = self.task_memory_path
        directory = os.path.dirname(path) or "."
        try:
            if not os.path.isdir(directory):
                os.makedirs(directory)
            trimmed = list(records)[-max(1, self.task_memory_max_records):]
            fd, tmp_path = tempfile.mkstemp(
                prefix=".task_memory.", suffix=".json", dir=directory
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(trimmed, f, ensure_ascii=False, indent=2)
                    f.write("\n")
                os.replace(tmp_path, path)
            finally:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
        except Exception as exc:
            rospy.logwarn("failed to save task memory %s: %s", path, exc)

    def _remember_task(self, command, understanding, result, reason):
        understanding = understanding if isinstance(understanding, dict) else {}
        intent = str(understanding.get("intent") or "unknown").strip()
        target = str(understanding.get("target_name") or "").strip()
        source = str(understanding.get("source_location") or "").strip()
        destination = str(
            understanding.get("destination_location")
            or understanding.get("delivery_target")
            or ""
        ).strip()
        placement_reference = str(understanding.get("placement_reference") or "").strip()
        placement_side = str(understanding.get("placement_side") or "").strip()
        result_text = "success" if result == "success" else "failed"
        reason = str(reason or "").strip()
        summary = self._summarize_task_record(
            command,
            intent,
            target,
            source,
            destination,
            placement_reference,
            placement_side,
            result_text,
            reason,
        )
        record = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "command": str(command or ""),
            "intent": intent,
            "target": target,
            "source_location": source,
            "destination_location": destination,
            "placement_reference": placement_reference,
            "placement_side": placement_side,
            "result": result_text,
            "reason": reason,
            "held_object_after": self._held_object,
            "summary": summary,
        }
        records = self._load_task_memory()
        records.append(record)
        self._save_task_memory(records)
        rospy.loginfo("task memory recorded: %s", json.dumps(record, ensure_ascii=False, sort_keys=True))

    def _summarize_task_record(
        self,
        command,
        intent,
        target,
        source,
        destination,
        placement_reference,
        placement_side,
        result,
        reason,
    ):
        target_text = target or "任务"
        if intent == "chat":
            action = "闲聊"
        elif intent == "come_to_speaker":
            action = "到说话人附近"
        elif intent == "navigate_to":
            action = "去%s" % (destination or "目标位置")
        elif intent == "transfer_object":
            if placement_reference and placement_side not in ("", "none"):
                action = "把%s放到%s%s" % (
                    target_text,
                    placement_reference,
                    _placement_side_text(placement_side),
                )
            else:
                action = "搬运%s到%s" % (target_text, destination or "目标位置")
        elif intent == "deliver_to_person":
            action = "拿%s给%s" % (target_text, destination or "目标人物")
        elif intent == "fetch_to_speaker":
            action = "拿%s" % target_text
            if source:
                action = "去%s%s" % (source, action)
        else:
            action = str(command or target_text)
        if result == "success":
            return "%s成功" % action
        if reason:
            return "%s失败，原因：%s" % (action, reason)
        return "%s失败" % action

    def _build_recent_task_context(self):
        count = max(0, int(self.task_memory_context_count))
        if count <= 0:
            return ""
        records = self._load_task_memory()
        recent = [item for item in records if isinstance(item, dict)][-count:]
        lines = []
        if recent:
            lines.append("最近任务上下文，只用于理解代词、刚才、再试一次、换一个等表达，不要覆盖用户当前明确指令：")
            for item in recent:
                summary = str(item.get("summary") or "").strip()
                held_after = str(item.get("held_object_after") or "").strip()
                if held_after:
                    summary = "%s；当时夹持：%s" % (summary, held_after)
                if summary:
                    lines.append("- %s" % summary)
        lines.append("当前夹持：%s" % (self._held_object or "无"))
        return "\n".join(lines)

    def _query_memory(self, command, understanding=None):
        request_id = str(int(time.time() * 1000))
        target_name = ""
        semantic_hint = ""
        source_location = ""
        source_point_id = ""
        query = command
        if isinstance(understanding, dict):
            target_name = str(understanding.get("target_name") or "").strip()
            semantic_hint = str(understanding.get("semantic_hint") or "").strip()
            source_location = str(understanding.get("source_location") or "").strip()
            if source_location:
                source_point_id, _ = self._resolve_point(source_location)
            parts = [target_name, semantic_hint, source_location, command]
            query = " ".join(part for part in parts if part)
        payload = {
            "request_id": request_id,
            "query": query,
            "user_command": command,
            "target_name": target_name,
            "semantic_hint": semantic_hint,
            "source_location": source_location,
            "source_point_id": source_point_id,
            "task_understanding": understanding or {},
            "max_results": 5,
        }
        if source_location:
            rospy.loginfo(
                "memory query source constraint: location=%s point_id=%s",
                source_location,
                source_point_id or "<unresolved>",
            )
        with self._memory_condition:
            self._pending_memory_request_id = request_id
            self._memory_result = None
        self.memory_query_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        return self._wait_json_result(
            self._memory_condition,
            lambda: self._memory_result,
            self.memory_query_timeout_sec,
        )

    def _confirm_target(self, target_id, target_name, command, target_obj=None):
        payload = {
            "target_id": target_id,
            "target_name": target_name,
            "user_query": command,
            "capture_count": self.capture_count,
        }
        if isinstance(target_obj, dict):
            payload["target_obj"] = target_obj
        with self._target_condition:
            self._target_result = None
        self.target_confirm_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        return self._wait_json_result(
            self._target_condition,
            lambda: self._target_result,
            self.target_confirm_timeout_sec,
        )

    def _select_tracking_target(self, target_id, target_name, command, target_obj=None):
        payload = {
            "target_id": target_id,
            "target_name": target_name,
            "user_query": command,
        }
        if isinstance(target_obj, dict):
            payload["target_obj"] = target_obj
        with self._select_condition:
            self._select_result = None
        self.memory_select_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        return self._wait_json_result(
            self._select_condition,
            lambda: self._select_result,
            self.memory_select_timeout_sec,
        )

    def _wait_tracker_lock(self):
        deadline = time.monotonic() + self.tracker_lock_timeout_sec
        with self._tracker_condition:
            self._tracker_status = None
            while not rospy.is_shutdown():
                status = self._tracker_status
                if isinstance(status, dict) and status.get("selected_active"):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._tracker_condition.wait(timeout=min(remaining, 0.2))
        return False

    def _auto_aim(self, target_pixel_x=None, tol_pixel_fine=None, auto_aim_params=None):
        if self.dry_run:
            rospy.loginfo("dry_run: skip auto_aim")
            return True
        if target_pixel_x is None:
            target_pixel_x = self.auto_aim_center_pixel_x
        if tol_pixel_fine is None:
            tol_pixel_fine = self.auto_aim_center_tol_pixel_fine
        params = dict(self.auto_aim_grasp_params)
        if auto_aim_params:
            params.update(auto_aim_params)
        with self._align_condition:
            self._align_success_seen = False
            self._align_success_time = 0.0
            self._align_active_since = 0.0
        rospy.set_param(self.auto_aim_target_pixel_param, int(target_pixel_x))
        rospy.set_param(self.auto_aim_tol_pixel_fine_param, int(tol_pixel_fine))
        self._set_auto_aim_motion_params(params)
        rospy.loginfo(
            "auto_aim params: pixel=%d tol=%d target_dist=%.1fcm stage1=%.1fcm",
            int(target_pixel_x),
            int(tol_pixel_fine),
            float(params["target_dist_cm"]),
            float(params["stage1_dist_cm"]),
        )
        if not self._set_align(True):
            self._restore_auto_aim_defaults()
            return False
        with self._align_condition:
            self._align_active_since = time.monotonic()
        deadline = time.monotonic() + self.align_timeout_sec
        try:
            with self._align_condition:
                while not rospy.is_shutdown():
                    if self._align_success_seen:
                        min_valid_time = (
                            self._align_active_since
                            + max(0.0, self.align_min_success_delay_sec)
                        )
                        if self._align_success_time >= min_valid_time:
                            rospy.loginfo("auto_aim alignment succeeded")
                            return True
                        rospy.logwarn(
                            "ignore stale/early auto_aim success: age=%.3fs min=%.3fs",
                            self._align_success_time - self._align_active_since,
                            max(0.0, self.align_min_success_delay_sec),
                        )
                        self._align_success_seen = False
                        self._align_success_time = 0.0
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        rospy.logwarn("auto_aim alignment timeout")
                        return False
                    self._align_condition.wait(timeout=min(remaining, 0.2))
        finally:
            self._set_align(False)
            self._restore_auto_aim_defaults()

    def _set_auto_aim_motion_params(self, params):
        rospy.set_param(self.auto_aim_target_dist_cm_param, float(params["target_dist_cm"]))
        rospy.set_param(self.auto_aim_stage1_dist_cm_param, float(params["stage1_dist_cm"]))
        rospy.set_param(self.auto_aim_tol_dist_stage1_param, float(params["tol_dist_stage1"]))
        rospy.set_param(self.auto_aim_tol_dist_final_param, float(params["tol_dist_final"]))
        rospy.set_param(self.auto_aim_tol_yaw_param, float(params["tol_yaw"]))
        rospy.set_param(self.auto_aim_tol_pixel_rough_param, int(params["tol_pixel_rough"]))
        rospy.set_param(self.auto_aim_min_vel_y_param, float(params["min_vel_y"]))
        rospy.set_param(self.auto_aim_min_vel_z_param, float(params["min_vel_z"]))

    def _restore_auto_aim_defaults(self):
        rospy.set_param(self.auto_aim_target_pixel_param, self.auto_aim_center_pixel_x)
        rospy.set_param(
            self.auto_aim_tol_pixel_fine_param,
            self.auto_aim_center_tol_pixel_fine,
        )
        self._set_auto_aim_motion_params(self.auto_aim_grasp_params)

    def _navigate_to_point(self, point_id):
        point = self._patrol_points.get(point_id)
        if point is None:
            rospy.logwarn("point_id=%s not found in patrol points", point_id)
            return False
        return self._send_goal(point, self.nav_timeout_sec)

    def _send_goal(self, point, timeout_sec):
        if self.dry_run:
            rospy.loginfo("dry_run: skip move_base goal for point=%s", point.get("point_id"))
            return True
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("move_base server is not available")
            return False

        pose = point.get("pose", point)
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        if "qz" in pose and "qw" in pose:
            qz = float(pose.get("qz", 0.0))
            qw = float(pose.get("qw", 1.0))
        else:
            yaw = float(pose.get("yaw", 0.0))
            qz = math.sin(yaw * 0.5)
            qw = math.cos(yaw * 0.5)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        rospy.loginfo("move_base goal: x=%.3f y=%.3f qz=%.3f qw=%.3f", x, y, qz, qw)
        self.move_base.send_goal(goal)
        ok = self.move_base.wait_for_result(rospy.Duration(timeout_sec))
        if not ok:
            rospy.logwarn("move_base goal timeout after %.1fs; canceling", timeout_sec)
            self.move_base.cancel_goal()
            return False
        state = self.move_base.get_state()
        rospy.loginfo("move_base result state=%s for point=%s", state, point.get("point_id"))
        return state == GoalStatus.SUCCEEDED

    def _navigate_back_to_person(self, person_pose):
        if self.dry_run:
            rospy.loginfo("dry_run: skip return navigation")
            return True
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("move_base server is not available")
            return False

        goal_x = person_pose.pose.position.x
        goal_y = person_pose.pose.position.y
        goal_yaw = 0.0
        recorded_x = goal_x
        recorded_y = goal_y
        rospy.loginfo(
            "return target source person pose: frame=%s x=%.3f y=%.3f standoff=%.3f arrival_radius=%.3f",
            person_pose.header.frame_id,
            recorded_x,
            recorded_y,
            self.return_standoff_m,
            self.return_arrival_radius_m,
        )
        if self.return_standoff_m > 0.0:
            try:
                robot_x, robot_y, _ = self._lookup_base_pose()
                dx = goal_x - robot_x
                dy = goal_y - robot_y
                dist = math.hypot(dx, dy)
                if dist > self.return_standoff_m:
                    goal_x -= self.return_standoff_m * dx / dist
                    goal_y -= self.return_standoff_m * dy / dist
                goal_yaw = math.atan2(
                    person_pose.pose.position.y - goal_y,
                    person_pose.pose.position.x - goal_x,
                )
                rospy.loginfo(
                    "return standoff applied: robot=(%.3f, %.3f) person=(%.3f, %.3f) goal=(%.3f, %.3f) dist_before=%.3f yaw=%.1fdeg",
                    robot_x,
                    robot_y,
                    recorded_x,
                    recorded_y,
                    goal_x,
                    goal_y,
                    dist,
                    math.degrees(goal_yaw),
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("return standoff skipped: cannot lookup current robot pose")

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        rospy.loginfo(
            "return move_base goal: frame=%s x=%.3f y=%.3f yaw=%.1fdeg q=(%.3f %.3f %.3f %.3f)",
            self.map_frame,
            goal_x,
            goal_y,
            math.degrees(goal_yaw),
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )
        self.move_base.send_goal(goal)
        deadline = time.monotonic() + self.return_timeout_sec
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.return_arrival_radius_m > 0.0:
                try:
                    robot_x, robot_y, _ = self._lookup_base_pose()
                    dist = math.hypot(
                        person_pose.pose.position.x - robot_x,
                        person_pose.pose.position.y - robot_y,
                    )
                    if dist <= self.return_arrival_radius_m:
                        self.move_base.cancel_goal()
                        self._stop_base()
                        rospy.loginfo(
                            "return succeeded by arrival radius: robot=(%.3f, %.3f) person=(%.3f, %.3f) dist=%.3f radius=%.3f",
                            robot_x,
                            robot_y,
                            person_pose.pose.position.x,
                            person_pose.pose.position.y,
                            dist,
                            self.return_arrival_radius_m,
                        )
                        return True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("return move_base succeeded with state=%s", state)
                return True
            if state in (
                GoalStatus.PREEMPTED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                rospy.logwarn("return move_base failed with terminal state=%s", state)
                return False
            rate.sleep()
        self.move_base.cancel_goal()
        rospy.logwarn("return move_base timeout after %.1fs; goal canceled", self.return_timeout_sec)
        return False

    def _set_align(self, enabled):
        if self.dry_run:
            rospy.loginfo("dry_run: set align %s", enabled)
            return True
        try:
            rospy.wait_for_service(self.align_service, timeout=self.service_timeout_sec)
            proxy = rospy.ServiceProxy(self.align_service, SetBool)
            response = proxy(bool(enabled))
            if not response.success:
                rospy.logwarn("align service returned failure: %s", response.message)
            return bool(response.success)
        except Exception as exc:
            rospy.logwarn("align service call failed: %s", exc)
            return False

    def _prepare_arm_before_task(self):
        self._publish_state("arm_init_pose_before_task")
        return self._call_trigger(self.arm_standby_service, "任务前机械臂初始姿态", required=True)

    def _call_trigger(self, service_name, label, required=True):
        if self.dry_run:
            rospy.loginfo("dry_run: skip service %s", service_name)
            return True
        try:
            rospy.wait_for_service(service_name, timeout=self.service_timeout_sec)
            proxy = rospy.ServiceProxy(service_name, Trigger)
            response = proxy()
            rospy.loginfo(
                "%s service response from %s: success=%s message=%s",
                label,
                service_name,
                bool(response.success),
                response.message,
            )
            if not response.success:
                rospy.logwarn("%s service returned failure: %s", label, response.message)
            return bool(response.success)
        except Exception as exc:
            rospy.logwarn("%s service call failed: %s", label, exc)
            return False if required else False

    def _wait_json_result(self, condition, getter, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        with condition:
            while not rospy.is_shutdown():
                value = getter()
                if value is not None:
                    return value
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                condition.wait(timeout=min(remaining, 0.2))
        return None

    def _on_memory_result(self, msg):
        data = self._parse_json_msg(msg, "memory query result")
        if data is None:
            return
        request_id = str(data.get("request_id") or "")
        with self._memory_condition:
            if self._pending_memory_request_id and request_id != self._pending_memory_request_id:
                return
            self._memory_result = data
            self._memory_condition.notify_all()

    def _on_understanding_result(self, msg):
        data = self._parse_json_msg(msg, "task understanding result")
        if data is None:
            return
        request_id = str(data.get("request_id") or "")
        with self._understanding_condition:
            if (
                self._pending_understanding_request_id
                and request_id != self._pending_understanding_request_id
            ):
                return
            self._understanding_result = data
            self._understanding_condition.notify_all()

    def _on_target_result(self, msg):
        data = self._parse_json_msg(msg, "target confirm result")
        if data is None:
            return
        with self._target_condition:
            self._target_result = data
            self._target_condition.notify_all()

    def _on_select_result(self, msg):
        data = self._parse_json_msg(msg, "memory select result")
        if data is None:
            return
        with self._select_condition:
            self._select_result = data
            self._select_condition.notify_all()

    def _on_tracker_status(self, msg):
        data = self._parse_json_msg(msg, "tracker status")
        if data is None:
            return
        with self._tracker_condition:
            self._tracker_status = data
            self._tracker_condition.notify_all()

    def _on_align_success(self, msg):
        if not msg.data:
            return
        with self._align_condition:
            self._align_success_seen = True
            self._align_success_time = time.monotonic()
            self._align_condition.notify_all()

    def _parse_json_msg(self, msg, label):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("payload must be an object")
            return data
        except Exception as exc:
            rospy.logwarn("ignore invalid %s JSON: %s", label, exc)
            return None

    def _trim_doa_samples(self, now):
        cutoff = now - max(self.doa_window_sec, 0.5)
        while self._doa_samples and self._doa_samples[0][0] < cutoff:
            self._doa_samples.popleft()

    def _normalize_doa_entries(self, entries):
        normalized = []
        for entry in entries:
            if (
                isinstance(entry, (tuple, list))
                and len(entry) >= 2
                and isinstance(entry[0], (int, float))
            ):
                try:
                    normalized.append((float(entry[0]), float(entry[1])))
                except Exception:
                    continue
            else:
                try:
                    normalized.append((None, float(entry)))
                except Exception:
                    continue
        return normalized

    def _drop_doa_edges(self, entries):
        if len(entries) < self.min_doa_samples:
            return entries
        timed = [entry for entry in entries if entry[0] is not None]
        if len(timed) != len(entries):
            return entries
        start = min(t for t, _ in entries)
        end = max(t for t, _ in entries)
        edge = max(self.doa_ignore_edge_sec, 0.0)
        if edge <= 0.0 or (end - start) <= edge * 2.0:
            return entries
        trimmed = [
            entry for entry in entries
            if (entry[0] - start) >= edge and (end - entry[0]) >= edge
        ]
        return trimmed if len(trimmed) >= self.min_doa_samples else entries

    def _densest_doa_cluster(self, corrected, window_deg):
        if not corrected:
            return [], None, 0.0
        window = min(max(float(window_deg), 1.0), 360.0)
        angles = sorted((float(deg) % 360.0) for deg in corrected)
        extended = angles + [deg + 360.0 for deg in angles]
        best_start = 0
        best_end = 0
        best_count = 0
        best_span = float("inf")
        end = 0
        for start in range(len(angles)):
            if end < start:
                end = start
            while end + 1 < start + len(angles) and extended[end + 1] - extended[start] <= window:
                end += 1
            count = end - start + 1
            span = extended[end] - extended[start]
            if count > best_count or (count == best_count and span < best_span):
                best_start = start
                best_end = end
                best_count = count
                best_span = span
        cluster = [normalize_deg(extended[i]) for i in range(best_start, best_end + 1)]
        selected = normalize_deg(circular_mean_deg(cluster)) if cluster else None
        ratio = float(best_count) / float(len(corrected))
        return cluster, selected, ratio

    def _most_frequent_doa(self):
        now = time.monotonic()
        with self._doa_lock:
            if self._latched_doa_samples is not None:
                entries = list(self._latched_doa_samples)
                self._latched_doa_samples = None
                rospy.loginfo(
                    "using %d latched DOA samples from session %s",
                    len(entries),
                    self._doa_session_id,
                )
            else:
                self._trim_doa_samples(now)
                entries = list(self._doa_samples)
        entries = self._normalize_doa_entries(entries)
        entries_before_trim = len(entries)
        entries = self._drop_doa_edges(entries)
        samples = [raw for _, raw in entries]
        if len(samples) < self.min_doa_samples:
            return None
        corrected = [self._correct_doa(raw) for raw in samples]
        bin_size = max(self.doa_bin_deg, 1.0)
        bins = Counter(int(round((deg % 360.0) / bin_size)) for deg in corrected)
        winner, winner_count = bins.most_common(1)[0]
        winner_ratio = float(winner_count) / float(len(corrected))
        min_winner_ratio = min(max(self.doa_min_winner_ratio, 0.0), 1.0)
        winner_center = (winner * bin_size) % 360.0
        in_winner = [
            deg for deg in corrected
            if abs(normalize_deg(deg - winner_center)) <= bin_size
        ]
        cluster_window = max(self.doa_cluster_window_deg, bin_size)
        cluster, selected, accept_ratio = self._densest_doa_cluster(
            corrected, cluster_window
        )
        cluster_count = len(cluster)
        if selected is None:
            mean = circular_mean_deg(in_winner) if in_winner else winner_center
            selected = normalize_deg(mean)
            cluster = [
                deg for deg in corrected
                if abs(normalize_deg(deg - selected)) <= cluster_window
            ]
            cluster_count = len(cluster)
            accept_ratio = float(len(cluster)) / float(len(corrected))
        try:
            raw_preview = ", ".join("%.1f" % float(v) for v in samples)
            corrected_preview = ", ".join("%.1f" % float(v) for v in corrected)
            cluster_preview = ", ".join("%.1f" % float(v) for v in cluster)
            bin_preview = ", ".join(
                "%d:%d" % (int(bin_id), int(count))
                for bin_id, count in bins.most_common()
            )
            rospy.loginfo(
                "DOA debug: raw=[%s] corrected=[%s] bin_size=%.1f bins={%s} "
                "winner_center=%.1f selected=%.1f winner_ratio=%.2f "
                "cluster_window=%.1f cluster=[%s] accept_ratio=%.2f threshold=%.2f "
                "cluster_count=%d min_samples=%d samples=%d/%d "
                "offset=%.1f ccw=%.1f",
                raw_preview,
                corrected_preview,
                bin_size,
                bin_preview,
                winner_center,
                selected,
                winner_ratio,
                cluster_window,
                cluster_preview,
                accept_ratio,
                min_winner_ratio,
                cluster_count,
                self.min_doa_samples,
                len(samples),
                entries_before_trim,
                self.doa_offset_deg,
                self.doa_ccw,
            )
        except Exception as exc:
            rospy.logwarn("DOA debug logging failed: %s", exc)
        if accept_ratio < min_winner_ratio or cluster_count < self.min_doa_samples:
            rospy.logwarn(
                "DOA rejected: accept_ratio=%.2f below threshold %.2f samples=%d "
                "cluster_count=%d min_samples=%d selected=%.1f window=%.1f",
                accept_ratio,
                min_winner_ratio,
                len(corrected),
                cluster_count,
                self.min_doa_samples,
                selected,
                cluster_window,
            )
            return None
        return selected

    def _correct_doa(self, raw_deg):
        return normalize_deg(float(raw_deg) * self.doa_ccw + self.doa_offset_deg)

    def _turn_by_relative_angle(self, doa_deg):
        if self.dry_run:
            rospy.loginfo("dry_run: skip turn %.1fdeg", doa_deg)
            return True
        try:
            _, _, start_yaw = self._lookup_base_pose()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("cannot lookup base yaw; skip turn: %s", exc)
            return False

        target_yaw = normalize_rad(start_yaw + math.radians(doa_deg))
        rate = rospy.Rate(20)
        deadline = time.monotonic() + self.turn_timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                _, _, current_yaw = self._lookup_base_pose()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue
            err = normalize_rad(target_yaw - current_yaw)
            err_deg = math.degrees(err)
            if abs(err_deg) <= self.turn_tolerance_deg:
                self._stop_base()
                return True
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
        return False

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

    def _current_robot_pose(self):
        try:
            x, y, yaw = self._lookup_base_pose()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def _lookup_base_pose(self):
        trans, rot = self.tf_listener.lookupTransform(
            self.map_frame,
            self.base_frame,
            rospy.Time(0),
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)
        return float(trans[0]), float(trans[1]), yaw

    def _load_patrol_points(self):
        try:
            with open(self.patrol_points_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            rospy.logwarn("failed to load patrol points: %s", exc)
            return {}
        if isinstance(data, dict):
            data = data.get("points", [])
        if not isinstance(data, list):
            rospy.logwarn("patrol points JSON must be a list or {'points': [...]}")
            return {}
        result = {}
        for point in data:
            point_id = str(point.get("point_id") or "").strip()
            if point_id:
                result[point_id] = point
        return result

    def _publish_turn_cmd(self, angular_z):
        cmd = Twist()
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    def _active_speech_profile(self):
        path = self.speech_profile_path
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            self._speech_profile = {}
            self._speech_profile_mtime = None
            return {}

        if self._speech_profile_mtime == mtime:
            return self._speech_profile

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
            requested = self.speech_profile_name or str(data.get("active_profile") or "default")
            profile = profiles.get(requested) or profiles.get("default") or {}
            if not isinstance(profile, dict):
                profile = {}
            self._speech_profile = profile
            self._speech_profile_mtime = mtime
            rospy.loginfo("speech profile loaded: %s from %s", requested, path)
        except Exception as exc:
            rospy.logwarn("failed to load speech profile %s: %s", path, exc)
            self._speech_profile = {}
            self._speech_profile_mtime = mtime
        return self._speech_profile

    def _load_persona_config(self):
        try:
            with open(self.speech_profile_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                data.setdefault("profiles", {})
                return data
        except Exception:
            pass
        return {"active_profile": "default", "profiles": {}}

    def _write_persona_config(self, data):
        directory = os.path.dirname(self.speech_profile_path) or "."
        if not os.path.isdir(directory):
            os.makedirs(directory)
        fd, tmp_path = tempfile.mkstemp(
            prefix=".voice_persona.", suffix=".json", dir=directory
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                f.write("\n")
            os.replace(tmp_path, self.speech_profile_path)
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)

    @staticmethod
    def _clean_persona_text(value, limit=120):
        text = str(value or "").strip()
        text = " ".join(text.split())
        return text[:limit]

    @staticmethod
    def _format_persona_template(template, fallback, style):
        text = str(template or fallback)
        try:
            return text.format(style=style)
        except Exception:
            return fallback.format(style=style)

    def _build_custom_profile(self, style, base, template=None):
        base = base if isinstance(base, dict) else {}
        template = template if isinstance(template, dict) else {}
        voice = str(base.get("tts_voice") or "Cherry")
        wake_replies = base.get("wake_replies", [])
        if not isinstance(wake_replies, list):
            wake_replies = []
        tts_style = self._format_persona_template(
            template.get("tts_style"),
            "{style}；适合家用取物机器人，发音清楚，语气自然。不要吓人，不要阴阳怪气，不要说脏话，不要承诺未完成的动作。",
            style,
        )
        task_instruction = self._format_persona_template(
            template.get("task_tts_instruction"),
            "tts_text 按这个自定义风格生成：{style}。必须保留任务事实，成功失败不能说反，中文短句，通常不超过30个汉字。",
            style,
        )
        phrase_instruction = self._format_persona_template(
            template.get("llm_phrase_instruction"),
            "{style}。保留事实，成功失败不能说反，不编造位置和物品，不要提系统细节，中文短句，通常不超过32个汉字。",
            style,
        )
        return {
            "description": str(template.get("description") or "App 自定义播报风格"),
            "tts_voice": voice,
            "tts_style": tts_style,
            "task_tts_instruction": task_instruction,
            "llm_phrase_instruction": phrase_instruction,
            "replacements": {},
            "wake_replies": wake_replies,
        }

    def _app_persona_options(self, data):
        profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
        configured = data.get("app_options", []) if isinstance(data, dict) else []
        options = []
        seen = set()
        if isinstance(configured, list):
            for item in configured:
                if not isinstance(item, dict):
                    continue
                option_id = self._clean_persona_text(
                    item.get("id") or item.get("profile") or "", limit=64
                )
                if not option_id or option_id in seen:
                    continue
                option_type = self._clean_persona_text(item.get("type") or "preset", limit=16)
                if option_type != "custom" and option_id not in profiles:
                    continue
                profile = profiles.get(option_id, {}) if isinstance(profiles, dict) else {}
                label = self._clean_persona_text(
                    item.get("label") or profile.get("description") or option_id,
                    limit=24,
                )
                options.append({
                    "id": option_id,
                    "label": label,
                    "type": "custom" if option_type == "custom" else "preset",
                    "description": str(profile.get("description") or item.get("description") or ""),
                })
                seen.add(option_id)
        for option_id in ("default", "calm_butler", "playful_partner", "chatty_funny"):
            if option_id in seen or option_id not in profiles:
                continue
            profile = profiles.get(option_id) or {}
            options.append({
                "id": option_id,
                "label": str(profile.get("description") or option_id),
                "type": "preset",
                "description": str(profile.get("description") or ""),
            })
            seen.add(option_id)
        if "custom" not in seen:
            profile = profiles.get("custom", {}) if isinstance(profiles, dict) else {}
            options.append({
                "id": "custom",
                "label": "自定义",
                "type": "custom",
                "description": str(profile.get("description") or "App 自定义播报风格"),
            })
        return options

    def _publish_persona_status(self, status, **extra):
        data = self._load_persona_config()
        profiles = data.get("profiles", {}) if isinstance(data, dict) else {}
        payload = {
            "status": status,
            "active_profile": data.get("active_profile", "default"),
            "profiles": sorted(profiles.keys()) if isinstance(profiles, dict) else [],
            "app_options": self._app_persona_options(data),
        }
        payload.update(extra)
        self.persona_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _say_key(self, key, fallback, **values):
        profile = self._active_speech_profile()
        phrases = profile.get("phrases", {}) if isinstance(profile, dict) else {}
        template = phrases.get(key, fallback) if isinstance(phrases, dict) else fallback
        try:
            text = str(template).format(**values)
        except Exception as exc:
            rospy.logwarn("speech template failed for key=%s: %s", key, exc)
            text = fallback.format(**values)
        generated = self._generate_llm_phrase(key, text, values, profile)
        if generated:
            text = generated
        self._say(text, allow_llm=False)

    def _say(self, text, allow_llm=True):
        if not text:
            return
        if not str(text).startswith("__local__:"):
            profile = self._active_speech_profile()
            replacements = profile.get("replacements", {}) if isinstance(profile, dict) else {}
            if isinstance(replacements, dict):
                text = replacements.get(text, text)
            if allow_llm:
                generated = self._generate_llm_phrase("status", text, {}, profile)
                if generated:
                    text = generated
        self.tts_pub.publish(String(data=text))

    def _generate_llm_phrase(self, key, base_text, values, profile):
        if not self.enable_llm_speech:
            return ""
        api_key = os.getenv(str(self.llm_speech_api_key_env), "").strip()
        if not api_key:
            return ""

        instruction = ""
        if isinstance(profile, dict):
            instruction = str(profile.get("llm_phrase_instruction") or "").strip()
        if not instruction:
            instruction = (
                "用自然、轻松的中文改写机器人播报，保持原意，不解释系统细节。"
            )

        target_name = str(values.get("target_name") or "").strip()
        context = {
            "state_key": key,
            "base_text": str(base_text),
            "target_name": target_name,
        }
        prompt = (
            "你是家用取物机器人的语音播报文案生成器。\n"
            "根据当前任务状态生成一句中文播报。\n"
            "要求：\n"
            "- 只输出一句话，不要 JSON，不要引号，不要解释。\n"
            "- 保持 base_text 的事实含义，不要承诺没有发生的动作。\n"
            "- 不要出现系统、节点、ROS、模型、接口、日志等技术词。\n"
            "- 可以每次换一种表达，允许轻微幽默，但不能影响用户理解。\n"
            "- 通常 8 到 32 个汉字；失败/取消类状态要清楚直接。\n"
            f"- 当前风格：{instruction}\n"
            f"上下文：{json.dumps(context, ensure_ascii=False)}\n"
            "播报："
        )

        payload = {
            "model": str(self.llm_speech_model),
            "messages": [
                {
                    "role": "system",
                    "content": "你只生成机器人要播放的一句中文播报。",
                },
                {"role": "user", "content": prompt},
            ],
            "temperature": self.llm_speech_temperature,
            "enable_thinking": False,
        }
        endpoint = self.llm_speech_base_url.rstrip("/") + "/chat/completions"
        request = urllib.request.Request(
            endpoint,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": "Bearer %s" % api_key,
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self.llm_speech_timeout_sec
            ) as response:
                body = json.loads(response.read().decode("utf-8"))
            text = body["choices"][0]["message"]["content"]
            return self._clean_llm_phrase(text)
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError, IndexError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(
                30.0, "LLM speech phrase failed; using fallback: %s", exc
            )
            return ""

    @staticmethod
    def _clean_llm_phrase(text):
        text = str(text or "").strip()
        if not text:
            return ""
        if text.startswith("```"):
            text = text.replace("```json", "").replace("```", "").strip()
        text = text.strip("\"'“”‘’ \n\t")
        for prefix in ("播报：", "播报:", "语音：", "语音:"):
            if text.startswith(prefix):
                text = text[len(prefix):].strip()
        text = text.splitlines()[0].strip()
        if not text or len(text) > 80:
            return ""
        blocked_terms = ("ROS", "ros", "节点", "模型", "接口", "日志", "API")
        if any(term in text for term in blocked_terms):
            return ""
        return text

    def _publish_state(self, state):
        rospy.loginfo("voice_fetch state -> %s", state)
        self.state_pub.publish(String(data=state))

    def run(self):
        rospy.spin()


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _placement_side_text(side):
    return {
        "left": "左边",
        "right": "右边",
        "front": "前面",
        "back": "后面",
    }.get(str(side or "").strip().lower(), "旁边")


def _semantic_hint_for_target(target):
    text = str(target or "")
    if any(term in text for term in ("矿泉水", "水", "饮料", "冰红茶", "茶", "可乐", "瓶", "农夫山泉")):
        return "矿泉水 饮料 瓶装饮料 bottle"
    if any(term in text for term in ("杯", "水杯", "杯子")):
        return "杯子 cup"
    if any(term in text for term in ("遥控器", "遥控")):
        return "遥控器 remote"
    if any(term in text for term in ("手机", "电话")):
        return "手机 cell phone"
    if any(term in text for term in ("书", "本子")):
        return "书 book"
    if any(term in text for term in ("快递", "包裹", "盒", "箱")):
        return "快递 包裹 package box"
    return text


def _reference_semantic_hint(name):
    text = str(name or "")
    if any(term in text for term in ("矿泉水", "水", "饮料", "茶", "可乐")):
        return "饮料 瓶装饮料 bottle"
    if any(term in text for term in ("杯", "水杯")):
        return "杯子 cup"
    if any(term in text for term in ("快递", "包裹", "盒", "箱")):
        return "快递 包裹 盒子 package box"
    return ""


def _target_yolo_classes(target_name, semantic_hint):
    text = ("%s %s" % (target_name or "", semantic_hint or "")).lower()
    if "bottle" in text or any(term in text for term in ("矿泉水", "瓶装", "饮料", "茶", "可乐", "水")):
        return "bottle", ["cup"]
    if any(term in text for term in ("喷雾", "喷雾罐", "喷雾剂", "罐")):
        return "bottle", ["cup"]
    if "cup" in text or any(term in text for term in ("杯", "水杯")):
        return "cup", ["bottle"]
    if "book" in text or "书" in text:
        return "book", []
    if "remote" in text or "遥控器" in text:
        return "remote", ["cell phone"]
    if "cell phone" in text or "手机" in text:
        return "cell phone", ["remote"]
    if "bowl" in text or "碗" in text:
        return "bowl", ["cup"]
    if "scissors" in text or "剪刀" in text:
        return "scissors", []
    if "keyboard" in text or "键盘" in text:
        return "keyboard", []
    if "mouse" in text or "鼠标" in text:
        return "mouse", ["remote"]
    if "bag_wrapper" in text or any(term in text for term in ("包装袋", "袋子", "袋")):
        return "bag_wrapper", ["box"]
    if "box" in text or any(term in text for term in ("快递", "包裹", "盒", "箱")):
        return "box", ["bag_wrapper", "book"]
    if "teddy bear" in text or any(term in text for term in ("玩偶", "娃娃", "毛绒")):
        return "teddy bear", []
    if "apple" in text or "苹果" in text:
        return "apple", ["banana"]
    if "banana" in text or "香蕉" in text:
        return "banana", ["apple"]
    return "", []


def _possible_names_for_target(target_name):
    text = str(target_name or "").strip()
    names = [text] if text else []
    if any(term in text for term in ("矿泉水", "水", "饮料", "茶", "可乐")):
        for name in ("瓶装水", "饮用水", "饮料"):
            if name not in names:
                names.append(name)
    if any(term in text for term in ("杯", "水杯")) and "杯子" not in names:
        names.append("杯子")
    return names


if __name__ == "__main__":
    try:
        VoiceFetchOrchestrator().run()
    except rospy.ROSInterruptException:
        pass
