#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
from collections import deque
from sensor_msgs.msg import LaserScan, Range  
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

from astra_common import simplePID


# ===== 常用调参区：普通抓取默认值 =====
# 相对放置的左右偏置由 voice_fetch_orchestrator.py 顶部控制。
DEFAULT_TARGET_PIXEL_X = 320
DEFAULT_TOL_PIXEL_FINE = 10
DEFAULT_TARGET_DIST_CM = 12.0
DEFAULT_STAGE1_DIST_CM = 16.0
DEFAULT_TOL_DIST_STAGE1 = 2.0
DEFAULT_TOL_DIST_FINAL = 1.5
DEFAULT_TOL_YAW = 5.0
DEFAULT_TOL_PIXEL_ROUGH = 30
DEFAULT_MIN_VEL_Y = 0.05
DEFAULT_MIN_VEL_Z = 0.10


class YoloPIDAligner:
    def __init__(self):
        rospy.init_node('yolo_pid_align_node', anonymous=True)
        self.is_shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

        # ==========================================
        # 参数配置区 
        # ==========================================

        # --- 目标与画面中心 ---
        self.default_target_pixel_x = int(rospy.get_param('~target_pixel_x', DEFAULT_TARGET_PIXEL_X))
        self.target_pixel_param = rospy.get_param('~target_pixel_param', '/auto_aim_target_pixel_x')
        self.target_pixel_x = self.default_target_pixel_x
        self.default_tol_pixel_fine = int(rospy.get_param('~tol_pixel_fine', DEFAULT_TOL_PIXEL_FINE))
        self.tol_pixel_fine_param = rospy.get_param('~tol_pixel_fine_param', '/auto_aim_tol_pixel_fine')
        self.target_dist_cm_param = rospy.get_param('~target_dist_cm_param', '/auto_aim_target_dist_cm')
        self.stage1_dist_cm_param = rospy.get_param('~stage1_dist_cm_param', '/auto_aim_stage1_dist_cm')
        self.tol_dist_stage1_param = rospy.get_param('~tol_dist_stage1_param', '/auto_aim_tol_dist_stage1')
        self.tol_dist_final_param = rospy.get_param('~tol_dist_final_param', '/auto_aim_tol_dist_final')
        self.tol_yaw_param = rospy.get_param('~tol_yaw_param', '/auto_aim_tol_yaw')
        self.tol_pixel_rough_param = rospy.get_param('~tol_pixel_rough_param', '/auto_aim_tol_pixel_rough')
        self.min_vel_y_param = rospy.get_param('~min_vel_y_param', '/auto_aim_min_vel_y')
        self.min_vel_z_param = rospy.get_param('~min_vel_z_param', '/auto_aim_min_vel_z')
        self.target_name = rospy.get_param('~target_name', 'meat')
        self.detection_topic = rospy.get_param('~detection_topic', '/tracked_yolov8/detections')
        self.min_detection_conf = float(rospy.get_param('~min_detection_conf', 0.05))

        # --- 物理距离目标 (厘米) ---
        self.default_target_dist_cm = float(rospy.get_param('~target_dist_cm', DEFAULT_TARGET_DIST_CM))
        self.default_stage1_dist_cm = float(rospy.get_param('~stage1_dist_cm', DEFAULT_STAGE1_DIST_CM))
        self.target_dist_cm = self.default_target_dist_cm
        self.stage1_dist_cm = self.default_stage1_dist_cm

        # --- 容差 / 死区参数 ---
        self.default_tol_dist_stage1 = float(rospy.get_param('~tol_dist_stage1', DEFAULT_TOL_DIST_STAGE1))
        self.default_tol_dist_final = float(rospy.get_param('~tol_dist_final', DEFAULT_TOL_DIST_FINAL))
        self.default_tol_yaw = float(rospy.get_param('~tol_yaw', DEFAULT_TOL_YAW))
        self.default_tol_pixel_rough = int(rospy.get_param('~tol_pixel_rough', DEFAULT_TOL_PIXEL_ROUGH))
        self.tol_dist_stage1 = self.default_tol_dist_stage1
        self.tol_dist_final = self.default_tol_dist_final
        self.tol_yaw = self.default_tol_yaw
        self.tol_pixel_rough = self.default_tol_pixel_rough
        self.tol_pixel_fine = self.default_tol_pixel_fine

        self.default_min_vel_y = float(rospy.get_param('~min_vel_y', DEFAULT_MIN_VEL_Y))
        self.default_min_vel_z = float(rospy.get_param('~min_vel_z', DEFAULT_MIN_VEL_Z))
        self.min_vel_y = self.default_min_vel_y
        self.min_vel_z = self.default_min_vel_z

        # --- 传感器滤波 ---
        self.tof_alpha = 0.3                         
        self.median_window = 2                       
        self.lidar_alpha = 0.5                       

        # --- 统一的滑动窗口参数设置 (帧数与达标比例) ---
        # 1. 第一阶段粗调距离稳态窗口 (要求较低，能快速切入转正)
        self.s1_window_size = 15                     
        self.s1_ratio_thresh = 0.4                   

        # 2. 第二阶段雷达转正稳态窗口 (需要较高稳定性)
        self.s2_window_size = 15                     
        self.s2_ratio_thresh = 0.4                   

        # 3. 第三阶段精调距离稳态窗口 (防冲过)
        self.s3_window_size = 15                     
        self.s3_ratio_thresh = 0.4                   

        # 4. 第四阶段精调像素稳态窗口 (最终锁定)
        self.s4_window_size = 15                     
        self.s4_ratio_thresh = 0.4                   

        # ==========================================
        # 内部变量与初始化 
        # ==========================================
        self.name_to_id = {'meat': 1, 'vegetable': 2, 'fruit': 3, 'drink': 4}

        # 传感器历史队列
        self.angle_history = deque(maxlen=self.median_window)
        
        # 状态机滑动窗口队列
        self.s1_history = deque(maxlen=self.s1_window_size)
        self.s2_history = deque(maxlen=self.s2_window_size)
        self.s3_history = deque(maxlen=self.s3_window_size)
        self.s4_history = deque(maxlen=self.s4_window_size)

        # PID 控制器
        self.pid_x = simplePID(kp=0.012, ki=0.0, kd=0.0015) 
        self.pid_y = simplePID(kp=0.001, ki=0.0, kd=0.005) 
        self.pid_yaw = simplePID(kp=0.02, ki=0.0, kd=0.005) 

        # ROS 接口
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.success_pub = rospy.Publisher('/red_align_success', Bool, queue_size=1)
        self.toggle_srv = rospy.Service('/enable_redalign', SetBool, self.toggle_cb)

        # 状态变量
        self.is_tracking_enabled = False
        self.align_done = False 
        self.smooth_cx, self.smooth_cy = None, None
        self.last_seen_time = None

        self.smooth_lidar_angle = None 
        self.lidar_angle_error = 0.0
        self.is_lidar_valid = False

        self.work_state = 1  
        self.current_tof_dist_cm = None               

        rospy.Subscriber(self.detection_topic, String, self.yolo_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/laser", Range, self.tof_callback, queue_size=1)

        rospy.loginfo("=======================================")
        rospy.loginfo("YOLO 视觉伺服节点已启动 (全状态滑动窗口版)")
        rospy.loginfo("视觉输入: %s | min_conf: %.2f | target_pixel_x: %d",
                      self.detection_topic, self.min_detection_conf, self.target_pixel_x)
        rospy.loginfo("目标距离: %.1f cm | 角度容差: %.1f deg | 像素精调容差: %d", 
                      self.target_dist_cm, self.tol_yaw, self.tol_pixel_fine)
        rospy.loginfo("=======================================")

    # ---------- 回调与滤波 ----------
    def yolo_callback(self, msg):
        if not self.is_tracking_enabled or self.is_shutting_down:
            return

        if getattr(self, 'align_done', False):
            self.success_pub.publish(True)
            self.cmd_pub.publish(Twist()) 
            rospy.loginfo_throttle(2.0, "[任务完成] 物理对齐完美完成，正在发送交接信号...")
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            rospy.logerr_throttle(1.0, f"JSON 解析失败: {e}")
            return

        target_found = False
        target_cx, target_cy = 0, 0
        valid_targets = []

        for det in detections:
            if float(det.get('conf', 0.0)) >= self.min_detection_conf:
                valid_targets.append(det)

        if len(valid_targets) > 0:
            valid_targets.sort(key=lambda b: b['cx'], reverse=True)
            target_cx = valid_targets[0]['cx']
            target_cy = valid_targets[0]['cy']
            self.target_name = valid_targets[0]['name']
            target_found = True

        if target_found:
            self.last_seen_time = rospy.Time.now()

            ALPHA = 0.4 
            if getattr(self, 'smooth_cx', None) is None:
                self.smooth_cx, self.smooth_cy = target_cx, target_cy
            else:
                self.smooth_cx = ALPHA * target_cx + (1 - ALPHA) * self.smooth_cx
                self.smooth_cy = ALPHA * target_cy + (1 - ALPHA) * self.smooth_cy

            final_cx = int(self.smooth_cx)

            rospy.loginfo_throttle(1.0, f"追踪目标 [{self.target_name}] | 像素X:{final_cx} | 当前状态:{self.work_state}")

            twist_cmd = self.compute_approach_velocity(final_cx)
            self.cmd_pub.publish(twist_cmd)
        else:
            if self.last_seen_time is not None:
                elapsed = (rospy.Time.now() - self.last_seen_time).to_sec()
                if elapsed > 1.0: 
                    self.work_state = 1 
                    self.smooth_cx, self.smooth_cy = None, None 
                    self.cmd_pub.publish(Twist()) 
                    rospy.logwarn_throttle(1.0, "视野中无达标物品，系统等待中...")

    def tof_callback(self, msg):
        raw_dist_m = msg.range 
        if np.isinf(raw_dist_m) or np.isnan(raw_dist_m):
            return
        if raw_dist_m > 0.01 and raw_dist_m < 8.0: 
            new_dist_cm = raw_dist_m * 100.0
            if self.current_tof_dist_cm is None:
                self.current_tof_dist_cm = new_dist_cm 
            else:
                self.current_tof_dist_cm = self.tof_alpha * new_dist_cm + (1 - self.tof_alpha) * self.current_tof_dist_cm

    def scan_callback(self, data):
        if not self.is_tracking_enabled or self.is_shutting_down or self.align_done:
            return

        ranges = np.array(data.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = 0 
        angles = data.angle_min + np.arange(len(ranges)) * data.angle_increment
        angles = (angles + np.pi) % (2 * np.pi) - np.pi 

        ROI_ANGLE = np.radians(13) 
        DIST_MIN = 0.05
        DIST_MAX = 0.3 

        mask = (np.abs(angles) <= ROI_ANGLE) & (ranges > DIST_MIN) & (ranges < DIST_MAX)
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) < 8:
            self.is_lidar_valid = False
            return

        sort_idx = np.argsort(valid_angles)
        valid_angles = valid_angles[sort_idx]
        valid_ranges = valid_ranges[sort_idx]
        diffs = np.abs(np.diff(valid_ranges))
        jump_indices = np.where(diffs > 0.05)[0] 
        if len(jump_indices) > 0:
            first_jump = jump_indices[0]
            valid_ranges = valid_ranges[:first_jump + 1]
            valid_angles = valid_angles[:first_jump + 1]

        if len(valid_ranges) < 8:
            self.is_lidar_valid = False
            return

        X = valid_ranges * np.cos(valid_angles) 
        Y = valid_ranges * np.sin(valid_angles) 
        k, _ = np.polyfit(Y, X, 1)
        raw_angle = np.degrees(np.arctan(k))

        if abs(raw_angle) > 35.0:
            self.is_lidar_valid = False
            return

        self.angle_history.append(raw_angle)
        median_angle = np.median(self.angle_history)

        if self.smooth_lidar_angle is None:
            self.smooth_lidar_angle = median_angle
        else:
            self.smooth_lidar_angle = self.lidar_alpha * median_angle + (1 - self.lidar_alpha) * self.smooth_lidar_angle

        self.lidar_angle_error = self.smooth_lidar_angle
        self.is_lidar_valid = True

    def toggle_cb(self, req):
        if req.data:
            self.is_tracking_enabled = False
            self.target_pixel_x = int(rospy.get_param(
                self.target_pixel_param,
                self.default_target_pixel_x,
            ))
            self.tol_pixel_fine = int(rospy.get_param(
                self.tol_pixel_fine_param,
                self.default_tol_pixel_fine,
            ))
            self.target_dist_cm = float(rospy.get_param(
                self.target_dist_cm_param,
                self.default_target_dist_cm,
            ))
            self.stage1_dist_cm = float(rospy.get_param(
                self.stage1_dist_cm_param,
                self.default_stage1_dist_cm,
            ))
            self.tol_dist_stage1 = float(rospy.get_param(
                self.tol_dist_stage1_param,
                self.default_tol_dist_stage1,
            ))
            self.tol_dist_final = float(rospy.get_param(
                self.tol_dist_final_param,
                self.default_tol_dist_final,
            ))
            self.tol_yaw = float(rospy.get_param(
                self.tol_yaw_param,
                self.default_tol_yaw,
            ))
            self.tol_pixel_rough = int(rospy.get_param(
                self.tol_pixel_rough_param,
                self.default_tol_pixel_rough,
            ))
            self.min_vel_y = float(rospy.get_param(
                self.min_vel_y_param,
                self.default_min_vel_y,
            ))
            self.min_vel_z = float(rospy.get_param(
                self.min_vel_z_param,
                self.default_min_vel_z,
            ))
            rospy.loginfo(
                "收到唤醒指令，开始锁定目标。target_pixel_x=%d tol_pixel_fine=%d target_dist=%.1fcm stage1=%.1fcm",
                self.target_pixel_x,
                self.tol_pixel_fine,
                self.target_dist_cm,
                self.stage1_dist_cm,
            )
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
            self.align_done = False
            self.smooth_cx, self.smooth_cy = None, None
            self.last_seen_time = None
            self.smooth_lidar_angle = None 
            self.current_tof_dist_cm = None 
            self.is_lidar_valid = False
            self.work_state = 1
            
            # 清空所有滑动窗口历史记录
            self.angle_history.clear()
            self.s1_history.clear()
            self.s2_history.clear()
            self.s3_history.clear()
            self.s4_history.clear()
            self.is_tracking_enabled = True
        else:
            self.is_tracking_enabled = False
            self.align_done = False
            rospy.loginfo("收到休眠指令，已锁死底盘。")
            self.cmd_pub.publish(Twist())
        return SetBoolResponse(success=True, message="Status Changed")

    def shutdown_hook(self):
        self.is_shutting_down = True
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.05)

    # ---------- 统一滑动窗口判别函数 ----------
    def check_sliding_window(self, history_deque, window_size, threshold, condition_met):
        history_deque.append(condition_met)
        if len(history_deque) == window_size:
            ratio = sum(history_deque) / window_size
            return ratio >= threshold, ratio
        return False, 0.0

    # ---------- 核心状态机 ----------
    def compute_approach_velocity(self, current_x):
        twist = Twist()
        error_pixel_y = self.target_pixel_x - current_x    

        if self.current_tof_dist_cm is None:
            rospy.logwarn_throttle(1.0, "等待 ToF 距离传感器数据...")
            return twist

        if self.current_tof_dist_cm < (self.target_dist_cm - 5.0):
            twist.linear.x = -0.05
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.work_state = 1 

            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()

            self.s1_history.clear()
            self.s2_history.clear()
            self.s3_history.clear()
            self.s4_history.clear()

            rospy.logwarn_throttle(0.5, "距离过近，执行后退恢复，距离: %.1f cm", self.current_tof_dist_cm)
            return twist

        # ==========================================
        # 状态 1: 粗调距离与像素 (第一阶段前进)
        # ==========================================
        if self.work_state == 1:
            err_dist_1 = self.current_tof_dist_cm - self.stage1_dist_cm
            
            dist_ok = err_dist_1 <= self.tol_dist_stage1
            
            if not dist_ok:
                vel_x = self.pid_x.compute(0, -err_dist_1) 
                twist.linear.x = max(min(vel_x, 0.15), -0.15)
            
            if abs(error_pixel_y) > self.tol_pixel_rough:
                vel_y = self.pid_y.compute(0, -error_pixel_y)
                twist.linear.y = max(min(vel_y, 0.15), -0.15)

            rospy.loginfo_throttle(0.5, "[状态 1] 第一段安全前进中... 距换挡点误差: %.1fcm", err_dist_1)

            # 滑动窗口检测
            passed, ratio = self.check_sliding_window(self.s1_history, self.s1_window_size, self.s1_ratio_thresh, dist_ok)
            if passed:
                rospy.loginfo("达到换挡点稳态 (比例 %.0f%%)，进入雷达转正！", ratio*100)
                self.work_state = 2
                self.s2_history.clear()

        # ==========================================
        # 状态 2: 雷达平行转正
        # ==========================================
        elif self.work_state == 2:
            if not self.is_lidar_valid:
                rospy.logwarn_throttle(0.5, "等待有效雷达数据...")
                return twist

            angle_ok = abs(self.lidar_angle_error) <= self.tol_yaw

            if not angle_ok:
                raw_vel_z = self.pid_yaw.compute(0, self.lidar_angle_error)
                if raw_vel_z > 0 and raw_vel_z < self.min_vel_z:
                    vel_z = self.min_vel_z
                elif raw_vel_z < 0 and raw_vel_z > -self.min_vel_z:
                    vel_z = -self.min_vel_z
                else:
                    vel_z = raw_vel_z
                twist.angular.z = max(min(vel_z, 0.3), -0.3)
            
            rospy.loginfo_throttle(0.5, "[状态 2] 原地自转平行中... 偏航误差: %.1f deg", self.lidar_angle_error)

            # 滑动窗口检测
            passed, ratio = self.check_sliding_window(self.s2_history, self.s2_window_size, self.s2_ratio_thresh, angle_ok)
            if len(self.s2_history) == self.s2_window_size:
                rospy.loginfo_throttle(0.5, "[状态 2] 稳态窗口达标率: %.0f%%", ratio*100)
            
            if passed:
                rospy.loginfo("雷达转正稳态达成 (比例 %.0f%%)，进入贴脸微调距离！", ratio*100)
                self.work_state = 3
                self.s3_history.clear()

        # ==========================================
        # 状态 3: 最终贴脸微调 (距离精调)
        # ==========================================
        elif self.work_state == 3:
            err_dist_final = self.current_tof_dist_cm - self.target_dist_cm
            
            dist_final_ok = abs(err_dist_final) <= self.tol_dist_final

            if not dist_final_ok:
                vel_x = self.pid_x.compute(0, -err_dist_final)
                twist.linear.x = max(min(vel_x, 0.12), -0.12) 

            rospy.loginfo_throttle(0.5, "[状态 3] 贴脸距离精调中... 距离误差: %.1fcm", err_dist_final)

            # 滑动窗口检测 (替代原有的 1秒 timer)
            passed, ratio = self.check_sliding_window(self.s3_history, self.s3_window_size, self.s3_ratio_thresh, dist_final_ok)
            if len(self.s3_history) == self.s3_window_size:
                rospy.loginfo_throttle(0.5, "[状态 3] 距离稳态窗口达标率: %.0f%%", ratio*100)

            if passed:
                rospy.loginfo("贴脸距离稳态达成 (比例 %.0f%%)，进入最终像素级横向对齐！", ratio*100)
                self.work_state = 4
                self.s4_history.clear()

        # ==========================================
        # 状态 4: 像素级精调 (最终锁定)
        # ==========================================
        elif self.work_state == 4:
            pixel_ok = abs(error_pixel_y) <= self.tol_pixel_fine

            if not pixel_ok:
                raw_vel_y = self.pid_y.compute(0, -error_pixel_y)
                if raw_vel_y > 0 and raw_vel_y < self.min_vel_y:
                    vel_y = self.min_vel_y
                elif raw_vel_y < 0 and raw_vel_y > -self.min_vel_y:
                    vel_y = -self.min_vel_y
                else:
                    vel_y = raw_vel_y
                twist.linear.y = max(min(vel_y, 0.15), -0.15)
            
            rospy.loginfo_throttle(0.5, "[状态 4] 消除像素级横向偏差... 误差: %d", error_pixel_y)

            # 滑动窗口检测
            passed, ratio = self.check_sliding_window(self.s4_history, self.s4_window_size, self.s4_ratio_thresh, pixel_ok)
            if len(self.s4_history) == self.s4_window_size:
                rospy.loginfo_throttle(0.5, "[状态 4] 横向对齐稳态达标率: %.0f%%", ratio*100)

            if passed:
                target_id = self.name_to_id.get(self.target_name, 1)
                rospy.set_param('/current_aruco_id', target_id)
                rospy.loginfo(f"[任务下发] 终极稳态锁定！目标为 {self.target_name} -> ID: {target_id}")
                self.align_done = True
                return Twist()

        return twist

if __name__ == '__main__':
    try:
        YoloPIDAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
