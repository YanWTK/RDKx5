#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import json
import os
import threading
from yahboomcar_msgs.msg import ArmJoint
from std_srvs.srv import Trigger, TriggerResponse
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest 


# ======================== 抓取轨迹回放参数区 ========================
# 轨迹文件默认从 nav_pkg/trajectories 目录读取。
GRASP_TRAJECTORY_FILE = "arm_trajectory.json"
GRASP_TRAJECTORY_SPEED = 1.8
GRASP_MAX_JOINT_SPEED = 120.0
GRASP_CONTROL_HZ = 10.0
GRASP_SMOOTH_WINDOW = 5
GRASP_MIN_RUNTIME = 100
GRASP_RUNTIME_SCALE = 1.0
# True 表示轨迹回放时隔一个点执行一个点，减少约一半指令量；False 表示正常逐点回放。
GRASP_SKIP_EVERY_OTHER_POINT = False
GRASP_PREPOSITION_TIME = 1200
GRASP_SETTLE_TIME = 0.25
GRASP_PLAYBACK_JOINT5 = False
GRASP_JOINT5_FIXED_ANGLE = 270.0
GRASP_PLAYBACK_GRIPPER = False
GRASP_GRIPPER_OPEN_ANGLE = 30.0
GRASP_JOINT_OFFSETS = [0.0, 0.0, 2.0, 4.0, 0.0, 0.0]
# 抓取轨迹回放只连续控制这些舵机；1/5/6号不参与高频轨迹回放，降低底层总线和舵机压力。
GRASP_REPLAY_JOINT_IDS = [2, 3, 4]

JOINT_LIMITS = [
    (0.0, 180.0),
    (0.0, 180.0),
    (0.0, 180.0),
    (0.0, 180.0),
    (0.0, 270.0),
    (30.0, 180.0),
]


def trajectory_dir():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(scripts_dir)
    return os.path.join(pkg_dir, "trajectories")


def resolve_trajectory_path(path):
    if os.path.isabs(path):
        return path
    base = trajectory_dir()
    if os.path.exists(os.path.join(base, path)):
        return os.path.join(base, path)
    if not path.endswith(".json") and os.path.exists(os.path.join(base, path + ".json")):
        return os.path.join(base, path + ".json")
    return os.path.join(base, path)


def clamp_angles(joints):
    clamped = []
    for value, (low, high) in zip(joints, JOINT_LIMITS):
        clamped.append(max(low, min(high, float(value))))
    return clamped


def apply_joint_playback_mask(joints):
    masked = [float(value) + offset for value, offset in zip(joints, GRASP_JOINT_OFFSETS)]
    if not GRASP_PLAYBACK_JOINT5:
        masked[4] = GRASP_JOINT5_FIXED_ANGLE
    if not GRASP_PLAYBACK_GRIPPER:
        masked[5] = GRASP_GRIPPER_OPEN_ANGLE
    return clamp_angles(masked)


def max_joint_delta(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def interpolate(a, b, ratio):
    return [x + (y - x) * ratio for x, y in zip(a, b)]


def moving_average(points, window):
    if window <= 1:
        return points
    half = window // 2
    smoothed = []
    count = len(points)
    for i, point in enumerate(points):
        if i == 0 or i == count - 1:
            smoothed.append(point)
            continue
        left = max(0, i - half)
        right = min(count, i + half + 1)
        group = points[left:right]
        joints = []
        for joint_index in range(6):
            joints.append(sum(p["joints"][joint_index] for p in group) / float(len(group)))
        smoothed.append({"t": point["t"], "joints": joints})
    return smoothed


def skip_every_other_point(points):
    if len(points) <= 2:
        return points
    skipped = points[::2]
    if skipped[-1] is not points[-1]:
        skipped.append(points[-1])
    return skipped


def apply_speed_profile(points, speed, max_joint_speed):
    if len(points) < 2:
        return points
    result = [{"t": 0.0, "joints": points[0]["joints"]}]
    t_out = 0.0
    speed = max(0.01, speed)
    for previous, current in zip(points[:-1], points[1:]):
        raw_dt = max(0.0, current["t"] - previous["t"])
        dt = raw_dt / speed
        if max_joint_speed > 0:
            delta = max_joint_delta(previous["joints"], current["joints"])
            dt = max(dt, delta / max_joint_speed)
        t_out += dt
        result.append({"t": t_out, "joints": current["joints"]})
    return result


def resample(points, hz):
    if hz <= 0 or len(points) < 2:
        return points
    duration = points[-1]["t"]
    step = 1.0 / hz
    result = []
    src_index = 0
    t = 0.0
    while t < duration:
        while src_index < len(points) - 2 and points[src_index + 1]["t"] < t:
            src_index += 1
        p0 = points[src_index]
        p1 = points[src_index + 1]
        span = max(1e-6, p1["t"] - p0["t"])
        ratio = max(0.0, min(1.0, (t - p0["t"]) / span))
        result.append({"t": t, "joints": interpolate(p0["joints"], p1["joints"], ratio)})
        t += step
    result.append({"t": duration, "joints": points[-1]["joints"]})
    return result


class ArmGraspNode:
    def __init__(self):
        rospy.init_node("arm_grasp_service_node", anonymous=False)
        self._grasp_lock = threading.Lock()
        
        # 初始化发布者
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        
        # 初始化读取真实角度的 Service Client
        rospy.loginfo("正在等待底层角度查询服务 /CurrentAngle ...")
        rospy.wait_for_service('/CurrentAngle')
        self.get_angle_client = rospy.ServiceProxy('/CurrentAngle', RobotArmArray)
        rospy.loginfo("/CurrentAngle 服务连接成功！")

        # 声明抓取、释放、收回、待机服务
        self.service_grasp   = rospy.Service('/execute_grasp',   Trigger, self.handle_grasp_request)
        self.service_release = rospy.Service('/execute_release', Trigger, self.handle_release_request)
        self.service_retract = rospy.Service('/execute_retract', Trigger, self.handle_retract_request)
        self.service_retract_140 = rospy.Service('/execute_retract_140', Trigger, self.handle_retract_140_request)
        self.service_standby = rospy.Service('/arm_standby',     Trigger, self.handle_standby_request)
        self.service_init_pose = rospy.Service('/arm_init_pose',  Trigger, self.handle_init_pose_request)
        self.service_grasp_ready = rospy.Service('/arm_grasp_ready', Trigger, self.handle_grasp_ready_request)
        
        self.standby_joints = [90.0, 180.0, 0.0, 0.0, 270.0, 30.0]
        self.grasp_trajectory = self.load_grasp_trajectory(GRASP_TRAJECTORY_FILE)
        
        rospy.loginfo("=======================================")
        rospy.loginfo("独立机械臂节点已就绪！可用服务：")
        rospy.loginfo("  /execute_grasp   - 智能/强制抓取")
        rospy.loginfo("  /execute_release - 释放")
        rospy.loginfo("  /execute_retract - 收回")
        rospy.loginfo("  /execute_retract_140 - 收回到1号舵机140度")
        rospy.loginfo("  /arm_standby     - 待机姿态")
        rospy.loginfo("  /arm_init_pose   - 启动初始化姿态")
        rospy.loginfo("  /arm_grasp_ready - 抓取轨迹起点")
        rospy.loginfo("=======================================")
        rospy.sleep(1.0)
        
        self.pubArm(self.standby_joints, run_time=2000)

    # ---------- 服务回调 ----------
    def handle_standby_request(self, req):
        rospy.loginfo("收到待机指令，开始执行...")
        self.execute_standby_sequence()
        res = TriggerResponse()
        res.success = True
        return res

    def handle_init_pose_request(self, req):
        rospy.loginfo("收到初始化姿态指令，回到启动初始化位置...")
        self.execute_init_pose_sequence()
        res = TriggerResponse()
        res.success = True
        return res

    def handle_grasp_ready_request(self, req):
        rospy.loginfo("收到抓取准备指令，移动到轨迹起点...")
        try:
            self.execute_grasp_ready_sequence()
            res = TriggerResponse()
            res.success = True
            return res
        except Exception as e:
            rospy.logerr("抓取准备错误: %s", str(e))
            res = TriggerResponse()
            res.success = False
            return res

    def handle_retract_request(self, req):
        rospy.loginfo("收到收回指令")
        self.execute_retract_sequence()
        res = TriggerResponse()
        res.success = True
        return res

    def handle_retract_140_request(self, req):
        rospy.loginfo("收到收回140指令")
        self.execute_retract_140_sequence()
        res = TriggerResponse()
        res.success = True
        return res

    def handle_release_request(self, req):
        rospy.loginfo("收到释放指令！")
        try:
            self.execute_release_sequence()
            res = TriggerResponse()
            res.success = True
            return res
        except Exception as e:
            res = TriggerResponse()
            res.success = False
            return res

    def handle_grasp_request(self, req):
        rospy.loginfo("收到抓取指令！开始执行...")
        if not self._grasp_lock.acquire(False):
            rospy.logwarn("抓取服务正忙，忽略重复 /execute_grasp 请求。")
            res = TriggerResponse()
            res.success = False
            res.message = "grasp_busy"
            return res
        try:
            grasp_ok = self.execute_grasp_sequence()
            res = TriggerResponse()
            res.success = bool(grasp_ok)
            res.message = "grasp_ok" if grasp_ok else "gripper did not confirm object"
            return res
        except Exception as e:
            rospy.logerr("抓取错误: %s", str(e))
            res = TriggerResponse()
            res.success = False
            res.message = str(e)
            return res
        finally:
            self._grasp_lock.release()

    # ---------- 底层发布函数 ----------
    def pubArm(self, joints=[], id=10, angle=90, run_time=500):
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0: 
            armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pub_Arm.publish(armjoint)

    def pubArm2(self, joints=[], id=10, angle=90, run_time=1000):
        self.pubArm(joints, id, angle, run_time)

    def read_gripper_angle(self, req, retries=3, retry_delay=0.04):
        last_error = None
        for _ in range(max(1, int(retries))):
            try:
                response = self.get_angle_client(req)
                if len(response.angles) < 6:
                    last_error = "CurrentAngle 返回关节数量不足: %d" % len(response.angles)
                else:
                    real_angle = float(response.angles[5])
                    if real_angle > 0:
                        return real_angle
                    last_error = "CurrentAngle 返回无效夹爪角度: %.1f" % real_angle
            except Exception as e:
                last_error = str(e)
            rospy.sleep(max(0.0, float(retry_delay)))
        rospy.logwarn("读取夹爪角度失败: %s", last_error)
        return None

    def pub_grasp_replay_point(self, joints, run_time):
        for joint_id in GRASP_REPLAY_JOINT_IDS:
            if joint_id < 1 or joint_id > len(joints):
                continue
            self.pubArm([], id=joint_id, angle=joints[joint_id - 1], run_time=run_time)

    def load_grasp_trajectory(self, filename):
        path = resolve_trajectory_path(filename)
        if not os.path.exists(path):
            rospy.logwarn("抓取轨迹文件不存在，将使用原来的手写抓取动作: %s", path)
            return None
        try:
            with open(path, "r") as f:
                data = json.load(f)
            points = data.get("points", [])
            if len(points) < 2:
                rospy.logwarn("抓取轨迹点少于 2 个，将使用原来的手写抓取动作: %s", path)
                return None

            normalized = []
            first_t = float(points[0]["t"])
            for point in points:
                joints = point.get("joints", [])
                if len(joints) < 6:
                    rospy.logwarn("抓取轨迹存在关节数量不足的点，将使用原来的手写抓取动作。")
                    return None
                normalized.append({
                    "t": max(0.0, float(point["t"]) - first_t),
                    "joints": apply_joint_playback_mask(joints[:6]),
                })

            normalized = moving_average(normalized, GRASP_SMOOTH_WINDOW)
            normalized = apply_speed_profile(normalized, GRASP_TRAJECTORY_SPEED, GRASP_MAX_JOINT_SPEED)
            normalized = resample(normalized, GRASP_CONTROL_HZ)
            if GRASP_SKIP_EVERY_OTHER_POINT:
                original_count = len(normalized)
                normalized = skip_every_other_point(normalized)
                rospy.loginfo("已开启隔点回放: %d -> %d 点", original_count, len(normalized))
            rospy.loginfo("已加载抓取轨迹: %s, 点数: %d, 时长: %.2f s", path, len(normalized), normalized[-1]["t"])
            return normalized
        except Exception as e:
            rospy.logwarn("加载抓取轨迹失败，将使用原来的手写抓取动作: %s", str(e))
            return None

    def replay_grasp_trajectory(self):
        if not self.grasp_trajectory:
            return False

        points = self.grasp_trajectory
        rospy.loginfo("开始回放抓取轨迹，仅控制舵机: %s", GRASP_REPLAY_JOINT_IDS)
        self.execute_grasp_ready_sequence()

        start = time.time()
        last_t = points[0]["t"]
        self.pub_grasp_replay_point(points[0]["joints"], run_time=GRASP_MIN_RUNTIME)
        for point in points[1:]:
            if rospy.is_shutdown():
                return False
            dt = max(0.0, point["t"] - last_t)
            run_time = int(max(GRASP_MIN_RUNTIME, round(dt * 1000.0 * GRASP_RUNTIME_SCALE)))
            self.pub_grasp_replay_point(point["joints"], run_time=run_time)

            sleep_time = start + point["t"] - time.time()
            if sleep_time > 0:
                rospy.sleep(sleep_time)
            last_t = point["t"]

        rospy.sleep(GRASP_SETTLE_TIME)
        rospy.loginfo("抓取轨迹回放完成。")
        return True

    def execute_grasp_ready_sequence(self):
        if not self.grasp_trajectory:
            self.pubArm(self.standby_joints, run_time=GRASP_PREPOSITION_TIME)
        else:
            self.pub_grasp_replay_point(self.grasp_trajectory[0]["joints"], run_time=GRASP_PREPOSITION_TIME)
        time.sleep(GRASP_PREPOSITION_TIME / 1000.0 + GRASP_SETTLE_TIME)

    def execute_builtin_grasp_approach(self):
        rospy.loginfo("第一步：下探准备")
        self.pubArm2([],id=6,angle=30) # 确保夹爪张开
        self.pubArm([],id=2,angle=145)
        rospy.loginfo("第二步：调整姿态")
        self.pubArm2([],id=3,angle=40)
        self.pubArm2([],id=5,angle=270)
        self.pubArm2([],id=3,angle=60)
        time.sleep(0.5)
        self.pubArm2([],id=4,angle=40)
        rospy.loginfo("第三步：靠近物品")
        self.pubArm2([],id=2,angle=70)
        time.sleep(1)

    # ======== 动作序列 ========
    def execute_release_sequence(self):
        rospy.loginfo("第一步")
        self.pubArm([],id=2,angle=145)
        rospy.loginfo("第二步")
        self.pubArm2([],id=3,angle=40)
        self.pubArm2([],id=5,angle=270)
        self.pubArm2([],id=3,angle=60) 
        time.sleep(1)
        self.pubArm2([],id=4,angle=50)
        rospy.loginfo("第三步")        
        self.pubArm2([],id=2,angle=60)
        time.sleep(1)
        self.pubArm([], id=6, angle=30, run_time=1200)
        time.sleep(0.6)
        self.pubArm([], id=6, angle=30, run_time=800)
        time.sleep(0.8)
        self.pubArm2([],id=2,angle=70)
        self.pubArm2([],id=2,angle=145)
        self.pubArm2([],id=3,angle=35)
        self.pubArm2([],id=1,angle=90)
        self.pubArm2([],id=3,angle=0)
        self.pubArm2([],id=4,angle=0)
        self.pubArm2([],id=2,angle=180)
        time.sleep(0.8)

    def execute_grasp_sequence(self):
        self.pubArm2([],id=6,angle=30) # 确保夹爪张开
        if not self.replay_grasp_trajectory():
            self.execute_builtin_grasp_approach()
        
        # =======================================================
        # [修改点] 获取开关状态，动态决定使用哪种抓取方式
        # =======================================================
        use_smart = rospy.get_param('/use_smart_gripper', True)

        if use_smart:
            # ---------------- 开启智能抓取 ----------------
            rospy.loginfo("开始智能收缩夹爪 (堵转检测模式)...")
            # 智能夹爪参数说明：
            # - open_angle：夹爪张开起始角度；闭合循环从这个角度开始。
            # - max_close_angle：最大闭合角度；循环到这里仍未确认夹住就判定失败。
            # - step_deg：每轮增加的闭合角度；越大夹得越快，但角度反馈更容易跳过细节。
            # - error_threshold_deg：堵转/夹到物体判定阈值；目标角度 - 实际角度超过该值，认为夹爪可能被物体挡住。
            # - min_real_progress_deg：真实角度至少要比闭合起点动过这么多，才允许确认夹住；
            #   避免夹爪命令没执行/反馈卡住时，在 30° 附近误判成功。
            # - squeeze_offset_deg：确认夹住后追加的施力角度，用来保持夹紧力。
            # - motion_check_*：疑似堵转后保持当前目标角度，连续复查真实角度是否还在运动；
            #   如果还在运动，说明只是反馈滞后，继续闭合；如果基本不动，才确认夹住。
            # - release_check_delta_deg：保留兼容参数；当前趋势复查逻辑不再主动回退夹爪。
            # - close_run_time_ms：每一步闭合命令的舵机运动时间；越小越快，太小会导致反馈滞后。
            # - close_settle_sec/min_motion_fraction_before_check：每一步闭合后至少等待舵机走完一部分行程再判断。
            # - confirm_run_time_ms：确认夹住后发送最终施力角度的舵机运动时间。
            # - read_retries/read_retry_delay_sec：读取 /CurrentAngle 的重试次数和间隔，用于过滤 0/-1 等无效反馈。
            current_target = float(rospy.get_param('/smart_gripper_open_angle', 30.0))
            max_close_angle = float(rospy.get_param('/smart_gripper_max_close_angle', 165.0))
            step = float(rospy.get_param('/smart_gripper_step_deg', 4.0))
            error_threshold = float(rospy.get_param('/smart_gripper_error_threshold_deg', 5.0))
            min_real_progress = float(rospy.get_param('/smart_gripper_min_real_progress_deg', 5.0))
            squeeze_offset = float(rospy.get_param('/smart_gripper_squeeze_offset_deg', 2.0))
            release_check_delta = float(rospy.get_param('/smart_gripper_release_check_delta_deg', 4.0))
            motion_check_samples = int(rospy.get_param('/smart_gripper_motion_check_samples', 6))
            motion_check_interval = float(rospy.get_param('/smart_gripper_motion_check_interval_sec', 0.12))
            motion_min_delta = float(rospy.get_param('/smart_gripper_motion_min_delta_deg', 1.0))
            close_run_time = int(rospy.get_param('/smart_gripper_close_run_time_ms', 250))
            close_settle_sec = float(rospy.get_param('/smart_gripper_close_settle_sec', 0.18))
            min_motion_fraction = float(rospy.get_param('/smart_gripper_min_motion_fraction_before_check', 0.8))
            confirm_run_time = int(rospy.get_param('/smart_gripper_confirm_run_time_ms', 600))
            read_retries = int(rospy.get_param('/smart_gripper_read_retries', 3))
            read_retry_delay = float(rospy.get_param('/smart_gripper_read_retry_delay_sec', 0.03))
            grasp_ok = False
            
            req = RobotArmArrayRequest()
            initial_real_angle = self.read_gripper_angle(req, read_retries, read_retry_delay)
            if initial_real_angle is None:
                initial_real_angle = current_target
            max_real_angle_seen = initial_real_angle
            
            while current_target <= max_close_angle and not rospy.is_shutdown():
                current_target += step
                # 给定目标角度
                self.pubArm([], id=6, angle=current_target, run_time=close_run_time)
                
                # 延时等待舵机运动。负载变高时反馈会落后，过早读取会把“正在合拢”误判成“夹住”。
                settle_time = max(close_settle_sec, (close_run_time / 1000.0) * min_motion_fraction)
                rospy.sleep(settle_time)
                
                # 读取当前真实角度
                real_angle = self.read_gripper_angle(req, read_retries, read_retry_delay)
                if real_angle is None:
                    continue
                max_real_angle_seen = max(max_real_angle_seen, real_angle)

                rospy.loginfo(
                    "夹爪闭合检测: target=%.1f real=%.1f diff=%.1f progress=%.1f",
                    current_target,
                    real_angle,
                    current_target - real_angle,
                    max_real_angle_seen - initial_real_angle,
                )

                # 如果真实角度比发送的指令角度小很多，说明被卡住了（夹到东西了）
                if (current_target - real_angle) >= error_threshold:
                    rospy.loginfo(
                        "检测到疑似阻挡，保持目标角度复查运动趋势: target=%.1f real=%.1f",
                        current_target,
                        real_angle,
                    )
                    trend_angles = [real_angle]
                    for _ in range(max(1, motion_check_samples)):
                        # 复查阶段只保持当前目标，不重复刷新同一个舵机命令，避免重置运动时间。
                        rospy.sleep(max(0.02, motion_check_interval))
                        trend_angle = self.read_gripper_angle(req, read_retries, read_retry_delay)
                        if trend_angle is not None:
                            trend_angles.append(trend_angle)
                            max_real_angle_seen = max(max_real_angle_seen, trend_angle)

                    moved_delta = max(trend_angles) - min(trend_angles)
                    latest_angle = trend_angles[-1]
                    still_blocked = (current_target - latest_angle) >= error_threshold
                    real_progress = max_real_angle_seen - initial_real_angle
                    rospy.loginfo(
                        "夹爪趋势复查: target=%.1f angles=%s moved=%.1f latest_diff=%.1f progress=%.1f",
                        current_target,
                        ",".join("%.1f" % value for value in trend_angles),
                        moved_delta,
                        current_target - latest_angle,
                        real_progress,
                    )

                    if moved_delta >= motion_min_delta or not still_blocked:
                        rospy.loginfo(
                            "疑似阻挡解除/反馈仍在运动，继续闭合: moved=%.1f latest=%.1f",
                            moved_delta,
                            latest_angle,
                        )
                        continue

                    if real_progress < min_real_progress:
                        rospy.logwarn(
                            "夹爪真实角度尚未有效移动，疑似反馈滞后/命令未执行，继续闭合: progress=%.1f need=%.1f",
                            real_progress,
                            min_real_progress,
                        )
                        continue
                    
                    # 确认真实角度不再明显变化，才认为夹到物体。
                    # 最终保持力基于真实接触角度追加补偿；current_target 只是检测压力，不代表真实接触点。
                    final_angle = min(latest_angle + squeeze_offset, max_close_angle)
                    rospy.loginfo(
                        "趋势确认夹取成功！初始角度: %.1f，目标角度: %.1f，最新角度: %.1f，施力角度: %.1f",
                        real_angle,
                        current_target,
                        latest_angle,
                        final_angle,
                    )
                    self.pubArm([], id=6, angle=final_angle, run_time=confirm_run_time)
                    rospy.sleep(max(0.35, (confirm_run_time / 1000.0) * 0.8))
                    grasp_ok = True
                    break
            
            if not grasp_ok:
                rospy.logwarn("智能夹爪未确认夹到物品，停止后续收回。current_target=%.1f max_close=%.1f", current_target, max_close_angle)
                return False
        else:
            # ---------------- 关闭智能抓取 (强制直接闭合) ----------------
            rospy.loginfo("开始强制闭合夹爪 (直接角度模式)...")
            # 设定直接闭合的角度。建议设为 165，保留一点空隙，防止在完全没有物品时电机硬怼死点烧毁。
            direct_close_angle = 170.0 
            self.pubArm([], id=6, angle=direct_close_angle, run_time=500)
            # 给机械臂 1 秒的时间去执行闭合动作
            rospy.sleep(1.0)
            rospy.loginfo("强制闭合动作执行完毕！")
        # =======================================================

        time.sleep(1)
        # 抓取复位
        self.pubArm2([],id=2,angle=70)
        self.pubArm2([],id=2,angle=145)
        self.pubArm2([],id=3,angle=35)
        self.pubArm2([],id=1,angle=90)
        self.pubArm2([],id=3,angle=5)
        self.pubArm2([],id=4,angle=15)
        self.pubArm2([],id=2,angle=180)
        time.sleep(0.5)
        self.pubArm2([],id=5,angle=270)
        return True

    def execute_retract_sequence(self):
        rospy.loginfo("机械臂正在收回...")
        self.pubArm2([],id=2,angle=70)
        self.pubArm2([],id=2,angle=145)
        self.pubArm2([],id=3,angle=35)
        self.pubArm2([],id=1,angle=90)
        self.pubArm2([],id=3,angle=5)
        self.pubArm2([],id=4,angle=15)
        self.pubArm2([],id=2,angle=180)
        self.pubArm2([],id=5,angle=270)

    def execute_retract_140_sequence(self):
        rospy.loginfo("机械臂正在收回到1号舵机140度...")
        self.pubArm2([],id=2,angle=70)
        self.pubArm2([],id=2,angle=145)
        self.pubArm2([],id=3,angle=35)
        self.pubArm2([],id=1,angle=170)
        self.pubArm2([],id=3,angle=0)
        self.pubArm2([],id=4,angle=0)
        self.pubArm2([],id=2,angle=170)
        self.pubArm2([],id=5,angle=270)

    def execute_init_pose_sequence(self):
        rospy.loginfo("执行启动初始化姿态...")
        self.pubArm(self.standby_joints, run_time=2000)
        rospy.sleep(2.0)

    def execute_standby_sequence(self):
        rospy.loginfo("执行待机动作序列...")
        self.pubArm2([],id=2,angle=144)
        self.pubArm2([],id=3,angle=30)
        self.pubArm2([],id=4,angle=0)
        self.pubArm2([],id=5,angle=90)


if __name__ == '__main__':
    try:
        ArmGraspNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
