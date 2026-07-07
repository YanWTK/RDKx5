#!/usr/bin/env bash
set -eo pipefail

APP_LOCK_FILE="${APP_LOCK_FILE:-/tmp/robopilot_app.lock}"
exec 9>"$APP_LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: Robopilot App stack is already running (lock: $APP_LOCK_FILE)." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
fi
source /opt/xiaogua/ros2_ws/install/setup.bash
export PYTHONPATH="/opt/xiaogua/ros2_ws/src/robopilot_app_bridge/src:${PYTHONPATH:-}"

ENABLE_CPU_AFFINITY="${ENABLE_CPU_AFFINITY:-true}"
NAV_CPUSET="${NAV_CPUSET:-0,1}"
VISION_CPUSET="${VISION_CPUSET:-3-7}"
ASTRA_COLOR_WIDTH="${ASTRA_COLOR_WIDTH:-640}"
ASTRA_COLOR_HEIGHT="${ASTRA_COLOR_HEIGHT:-480}"
ASTRA_COLOR_FPS="${ASTRA_COLOR_FPS:-30}"
MJPEG_STREAM_WIDTH="${MJPEG_STREAM_WIDTH:-320}"
MJPEG_STREAM_HEIGHT="${MJPEG_STREAM_HEIGHT:-240}"
MJPEG_STREAM_FPS="${MJPEG_STREAM_FPS:-8.0}"
MJPEG_JPEG_QUALITY="${MJPEG_JPEG_QUALITY:-80}"
START_FRPC="${START_FRPC:-true}"
STOP_FRPC_ON_EXIT="${STOP_FRPC_ON_EXIT:-true}"
FRPC_SERVICE="${FRPC_SERVICE:-xiaogua-frpc}"
case "$ASTRA_COLOR_FPS" in
  30) ;;
  *)
    echo "WARN: ASTRA_COLOR_FPS=$ASTRA_COLOR_FPS 不在白名单内，回退到 30 (避免 astra_camera_node 启动崩溃)"
    ASTRA_COLOR_FPS=30
    ;;
esac
# 强制只接受 ORBBEC Astra Pro 实际验证支持的颜色 fps 值；
ASTRA_DEPTH_WIDTH="${ASTRA_DEPTH_WIDTH:-320}"
ASTRA_DEPTH_HEIGHT="${ASTRA_DEPTH_HEIGHT:-240}"
ASTRA_DEPTH_FPS="${ASTRA_DEPTH_FPS:-30}"
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-/opt/xiaogua/models/yolo_model.bin}"
YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-}"
YOLO_TARGET_CLASSES_WAS_SET="${YOLO_TARGET_CLASSES+x}"
YOLO_TARGET_CLASSES="${YOLO_TARGET_CLASSES:-person,bottle,cup,book,cell phone,remote,mouse,keyboard,clock,bowl,scissors}"
YOLO_CONF_THRESHOLD="${YOLO_CONF_THRESHOLD:-0.07}"
YOLO_NMS_THRESHOLD="${YOLO_NMS_THRESHOLD:-0.7}"
YOLO_MAX_FPS="${YOLO_MAX_FPS:-15.0}"
YOLO_PREPROCESS_MODE="${YOLO_PREPROCESS_MODE:-letterbox}"
if [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_alt.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_scene.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_good.bin" ]; then
  YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box}"
  if [ -z "$YOLO_TARGET_CLASSES_WAS_SET" ]; then
    YOLO_TARGET_CLASSES="person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"
  fi
fi
YOLO_CLASS_NAMES_ARGS=()
if [ -n "$YOLO_CLASS_NAMES" ]; then
  YOLO_CLASS_NAMES_ARGS=(yolo_class_names:="$YOLO_CLASS_NAMES")
fi
if [ "$ENABLE_CPU_AFFINITY" = "true" ] && { ! command -v taskset >/dev/null 2>&1 || [ "$(nproc)" -lt 8 ]; }; then
  echo "WARN: CPU affinity disabled (taskset missing or fewer than 8 CPUs)."
  ENABLE_CPU_AFFINITY=false
fi

configure_xvf3800_playback_volume() {
  if ! command -v amixer >/dev/null 2>&1; then
    echo "WARN: amixer not found; skip XVF3800 playback volume setup."
    return 0
  fi
  if amixer -c 0 info 2>/dev/null | grep -q "C16K6Ch"; then
    amixer -c 0 set PCM 100% >/dev/null 2>&1 || true
    amixer -c 0 set 'PCM',1 100% >/dev/null 2>&1 || true
    echo "XVF3800 playback volume set to 100% on card 0."
  else
    echo "WARN: XVF3800 card 0 not detected; skip playback volume setup."
  fi
}

DISABLE_USB_AUTOSUSPEND="${DISABLE_USB_AUTOSUSPEND:-true}"
if [ "$DISABLE_USB_AUTOSUSPEND" = "true" ]; then
  USB_AUTOSUSPEND_SCRIPT="/opt/xiaogua/ros2_ws/src/robopilot_app_bridge/scripts/disable_usb_autosuspend.sh"
  if [ -x "$USB_AUTOSUSPEND_SCRIPT" ]; then
    "$USB_AUTOSUSPEND_SCRIPT" || true
  elif [ -f "$USB_AUTOSUSPEND_SCRIPT" ]; then
    bash "$USB_AUTOSUSPEND_SCRIPT" || true
  else
    echo "WARN: USB autosuspend script not found: $USB_AUTOSUSPEND_SCRIPT"
  fi
fi

configure_xvf3800_playback_volume

TOPICS_GLOB='[/cmd_vel,/voltage,/odom,/scan,/cartographer_map,/map,/imu/imu_data,/diagnostics,/robot_status,/robot_pose,/voice_cmd,/voice_persona/set,/voice_persona/status,/move_base_simple/goal,/camera/rgb/image_raw,/camera/color/image_raw,/camera/depth/image_raw,/camera/depth/camera_info,/patrol_scan_cmd,/patrol_scan_done,/semantic_object_markers,/robopilot/mapping/control,/mode/status]'
SERVICES_GLOB='[/mapping/start,/mapping/save,/mapping/stop,/mode/switch_to_mapping,/mode/switch_to_navigation,/mode/switch_to_patrol,/mode/get_status]'
# 用 \" 包裹让 ROS2 把 [...] 当作字符串而非 YAML 数组
TOPICS_GLOB_ESC="\"${TOPICS_GLOB}\""
SERVICES_GLOB_ESC="\"${SERVICES_GLOB}\""

_ros1_has_ros_procs() {
  docker top xiaogua_runtime 2>/dev/null | grep -qvE 'defunct|^\s*$' && \
  docker top xiaogua_runtime 2>/dev/null | grep -qE '/opt/ros/|/catkin_ws/|roslaunch|rosrun|rosmaster|rosout|ydlidar|stp23l|cartographer|move_base|map_server|amcl|robopilot_ros1|mode_manager|base_node|robot_state|imu_filter|ekf_localization|joy_node|yahboom_joy|static_transform|laser_filter|apply_calib|Mcnamu'
}

_kill_ros1_all() {
  local signal="$1"
  docker exec xiaogua_runtime bash -lc "
    pkill -$signal -f '/opt/ros/' 2>/dev/null || true
    pkill -$signal -f '/catkin_ws/' 2>/dev/null || true
    pkill -$signal -f 'roslaunch' 2>/dev/null || true
    pkill -$signal -f 'rosmaster' 2>/dev/null || true
    pkill -$signal -f 'rosout' 2>/dev/null || true
  " >/dev/null 2>&1 || true
}

cleanup_ros1_container() {
  # Step 1: Ask Docker to send SIGINT to container PID 1 (roslaunch's graceful shutdown)
  docker kill --signal=SIGINT xiaogua_runtime 2>/dev/null || true
  sleep 2

  # Step 2: SIGTERM ALL ROS processes inside container (broad match covers orphaned nodes)
  _kill_ros1_all TERM

  # Step 3: Wait for roslaunch to gracefully shut down (up to 5s), then SIGKILL if needed
  local _i
  for _i in 1 2 3 4 5; do
    sleep 1
    if ! _ros1_has_ros_procs; then
      return 0
    fi
  done

  # Step 4: SIGKILL anything still alive
  _kill_ros1_all KILL
}

# Start a command in background, immune to SIGINT (only killed by cleanup via SIGTERM)
start_bg() {
  (trap '' INT; exec "$@") &
}

start_bg_on_cpus() {
  local cpus="$1"
  shift
  if [ "$ENABLE_CPU_AFFINITY" = "true" ]; then
    start_bg taskset -c "$cpus" "$@"
  else
    start_bg "$@"
  fi
}

start_frpc_service() {
  if [ "$START_FRPC" != "true" ]; then
    echo "FRPC tunnel disabled by START_FRPC=$START_FRPC"
    return 0
  fi
  if ! command -v systemctl >/dev/null 2>&1; then
    echo "WARN: systemctl not found; skip $FRPC_SERVICE startup."
    return 0
  fi
  if ! systemctl list-unit-files "$FRPC_SERVICE.service" >/dev/null 2>&1; then
    echo "WARN: $FRPC_SERVICE.service not found; skip FRPC tunnel startup."
    return 0
  fi
  systemctl start "$FRPC_SERVICE" || {
    echo "WARN: failed to start $FRPC_SERVICE;公网视频转发不会启用。"
    return 0
  }
  echo "FRPC tunnel started via $FRPC_SERVICE."
}

stop_frpc_service() {
  if [ "$START_FRPC" != "true" ] || [ "$STOP_FRPC_ON_EXIT" != "true" ]; then
    return 0
  fi
  if command -v systemctl >/dev/null 2>&1; then
    systemctl stop "$FRPC_SERVICE" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  trap - EXIT INT TERM
  stop_frpc_service
  cleanup_ros1_container

  # Kill all descendant processes (children + grandchildren of ros2 launch etc.)
  local pids
  pids="$(jobs -pr)" || true
  # Recursively collect all descendants of this shell
  local queue="$pids"
  while [ -n "$queue" ]; do
    local next_queue=""
    for pid in $queue; do
      local children
      children="$(pgrep -P "$pid" 2>/dev/null || true)"
      if [ -n "$children" ]; then
        pids="$pids $children"
        next_queue="$next_queue $children"
      fi
    done
    queue="$next_queue"
  done

  if [ -n "$pids" ]; then
    kill -TERM $pids >/dev/null 2>&1 || true
    # Wait for processes to exit gracefully (up to 5s), then SIGKILL if needed
    local _i _still_alive
    for _i in 1 2 3 4 5; do
      sleep 1
      _still_alive=""
      for _pid in $pids; do
        if kill -0 "$_pid" 2>/dev/null; then
          local _state
          _state="$(ps -o state= -p "$_pid" 2>/dev/null || true)"
          if [ "$_state" != "Z" ]; then
            _still_alive="$_still_alive $_pid"
          fi
        fi
      done
      pids="$_still_alive"
      if [ -z "$pids" ]; then
        break
      fi
    done
    if [ -n "$pids" ]; then
      kill -KILL $pids >/dev/null 2>&1 || true
    fi
    wait $(jobs -pr) >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

# 禁用 Realtek USB 摄像头（释放 USB 2.0 带宽）
REALTEK_DEV="/sys/bus/usb/devices/1-1.1.1/authorized"
if [ -f "$REALTEK_DEV" ] && [ "$(cat "$REALTEK_DEV")" = "1" ]; then
  echo 0 > "$REALTEK_DEV" 2>/dev/null && echo "Realtek camera disabled (USB bandwidth optimization)"
fi

start_bg_on_cpus "$VISION_CPUSET" python3 -m robopilot_app_bridge.mapping_service_node
MAPPING_PID=$!

start_bg_on_cpus "$VISION_CPUSET" python3 -m robopilot_app_bridge.mjpeg_server --ros-args \
  -p address:=0.0.0.0 \
  -p port:=8081 \
  -p stream_topic:=/camera/rgb/image_raw \
  -p source_topic:=/camera/color/image_raw \
  -p stream_fps:="$MJPEG_STREAM_FPS" \
  -p jpeg_quality:="$MJPEG_JPEG_QUALITY" \
  -p stream_width:="$MJPEG_STREAM_WIDTH" \
  -p stream_height:="$MJPEG_STREAM_HEIGHT"
MJPEG_PID=$!

start_frpc_service

start_bg_on_cpus "$NAV_CPUSET" python3 -m robopilot_app_bridge.cmd_vel_relay --ros-args \
  -p ws_port:=19091
CMDVEL_PID=$!

# Realtek Camera 已禁用（USB 1-1.1.1），Astra 作为唯一相机
start_bg_on_cpus "$VISION_CPUSET" ros2 launch astra_camera astra_pro.launch.xml \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  color_width:="$ASTRA_COLOR_WIDTH" \
  color_height:="$ASTRA_COLOR_HEIGHT" \
  color_fps:="$ASTRA_COLOR_FPS" \
  depth_width:="$ASTRA_DEPTH_WIDTH" \
  depth_height:="$ASTRA_DEPTH_HEIGHT" \
  depth_fps:="$ASTRA_DEPTH_FPS" \
  oni_log_to_console:=false \
  oni_log_to_file:=false
ASTRA_CAMERA_PID=$!

start_bg_on_cpus "$VISION_CPUSET" ros2 launch vlm_target_selector patrol_memory.launch.py \
  image_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/depth/camera_info \
  target_classes:="$YOLO_TARGET_CLASSES" \
  yolo_model_path:="$YOLO_MODEL_PATH" \
  "${YOLO_CLASS_NAMES_ARGS[@]}" \
  capture_count:=5 \
  capture_interval_sec:=0.25 \
  conf_threshold:="$YOLO_CONF_THRESHOLD" \
  yolo_nms_threshold:="$YOLO_NMS_THRESHOLD" \
  yolo_max_fps:="$YOLO_MAX_FPS" \
  yolo_preprocess_mode:="$YOLO_PREPROCESS_MODE" \
  yolo_task_gated:=true \
  process_only_when_scanning:=true \
  clear_memory_on_start:=false \
  use_local_vlm:=false \
  bailian_model:=qwen3-vl-plus \
  bailian_enable_thinking:=false \
  marker_text_scale:=0.07 \
  marker_text_z_offset:=0.10
PATROL_MEMORY_PID=$!

start_bg_on_cpus "$VISION_CPUSET" ros2 run rosapi rosapi_node --ros-args \
  -p topics_glob:="$TOPICS_GLOB_ESC" \
  -p services_glob:="$SERVICES_GLOB_ESC"
ROSAPI_PID=$!

start_bg_on_cpus "$VISION_CPUSET" ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p address:=0.0.0.0 \
  -p port:=9090 \
  -p default_call_service_timeout:=5.0 \
  -p call_services_in_new_thread:=true \
  -p send_action_goals_in_new_thread:=true \
  -p topics_glob:="$TOPICS_GLOB_ESC" \
  -p services_glob:="$SERVICES_GLOB_ESC"
ROSBRIDGE_PID=$!

# MQTT cloud bridge (optional — non-blocking, logs to file)
CLOUD_BRIDGE_ENV="/opt/xiaogua/ros2_ws/src/robopilot_app_bridge/config/mqtt_bridge.env"
if [ -f "$CLOUD_BRIDGE_ENV" ] && grep -q "MQTT_HOST=" "$CLOUD_BRIDGE_ENV" && ! grep -q "请填写" "$CLOUD_BRIDGE_ENV"; then
  mkdir -p /tmp/robopilot_app_logs
  start_bg_on_cpus "$NAV_CPUSET" python3 -m robopilot_app_bridge.robot_cloud_bridge \
    >> /tmp/robopilot_app_logs/robot_cloud_bridge.log 2>&1
  CLOUD_BRIDGE_PID=$!
  echo "MQTT cloud bridge started (PID=$CLOUD_BRIDGE_PID), log: /tmp/robopilot_app_logs/robot_cloud_bridge.log"
else
  echo "MQTT cloud bridge skipped (config not ready: $CLOUD_BRIDGE_ENV)"
fi

# 确保 Docker 容器运行
docker start xiaogua_runtime 2>/dev/null || true
sleep 2
cleanup_ros1_container

ROS1_CPU_ARGS=()
if [ "$ENABLE_CPU_AFFINITY" = "true" ]; then
  ROS1_CPU_ARGS=(taskset -c "$NAV_CPUSET")
fi
start_bg docker exec xiaogua_runtime "${ROS1_CPU_ARGS[@]}" bash -lc '
  set -eo pipefail

  cleanup_ros1() {
    for pid in ${BASE_PID:-} ${MODE_PID:-} ${BRIDGE_PID:-}; do
      if [ -n "$pid" ]; then
        kill "$pid" >/dev/null 2>&1 || true
      fi
    done
    wait ${BASE_PID:-} ${MODE_PID:-} ${BRIDGE_PID:-} >/dev/null 2>&1 || true
  }
  trap cleanup_ros1 EXIT INT TERM

  source /opt/ros/noetic/setup.bash
  source /opt/xiaogua/runtime_ws/yahboomcar_ws/devel/setup.bash
  source /opt/xiaogua/runtime_ws/software/library_ws/devel/setup.bash --extend
  source /opt/xiaogua/runtime_ws/software/carto_ws/install_isolated/setup.bash --extend
  export ROBOT_TYPE=X3plus
  export RPLIDAR_TYPE=X3
  export ROS_IP=$(hostname -I | awk "{print \$1}")

  for package in yahboomcar_nav ydlidar_ros_driver ldlidar cartographer_ros; do
    if ! rospack find "$package" >/dev/null; then
      echo "ERROR: ROS1 package not found: $package" >&2
      echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}" >&2
      exit 1
    fi
  done

  LOG_DIR=/tmp/robopilot_app_logs
  mkdir -p "$LOG_DIR"

  # Launch base hardware (lidar, motor driver, TF, laser filter)
  echo "ROS1 base hardware logs: $LOG_DIR/base.launch.log"
  roslaunch yahboomcar_nav base.launch >"$LOG_DIR/base.launch.log" 2>&1 &
  BASE_PID=$!

  # Wait for hardware to initialize
  sleep 8
  if ! kill -0 "$BASE_PID" >/dev/null 2>&1; then
    wait "$BASE_PID"
    exit 1
  fi

  # Launch mode manager (auto-starts navigation mode after 8s)
  roslaunch yahboomcar_nav mode_manager.launch &
  MODE_PID=$!
  sleep 2
  if ! kill -0 "$MODE_PID" >/dev/null 2>&1; then
    wait "$MODE_PID"
    exit 1
  fi

  # Launch ROS1 bridge (connects to ROS2 rosbridge)
  BRIDGE_LOG="$LOG_DIR/robopilot_ros1_app_bridge.log"
  # Rotate if >50MB: keep one backup, start fresh (so we never lose the last crash)
  if [ -f "$BRIDGE_LOG" ] && [ "$(stat -c%s "$BRIDGE_LOG" 2>/dev/null || echo 0)" -gt 52428800 ]; then
    mv "$BRIDGE_LOG" "${BRIDGE_LOG}.1"
  fi
  python3 /opt/xiaogua/runtime_ws/yahboomcar_ws/src/yahboomcar_nav/scripts/robopilot_ros1_app_bridge.py \
    _app_bridge_url:=ws://127.0.0.1:9090 \
    _publish_mock_topics:=true \
    _advertise_services:=true \
    _mapping_control_topic:=/robopilot/mapping/control \
    _publish_rate_hz:=5.0 \
    _scan_publish_hz:=3.0 \
    _map_publish_hz:=1.0 \
    _robot_pose_publish_hz:=5.0 \
    _publish_camera_over_rosbridge:=false \
    _max_send_queue:=200 >> "$BRIDGE_LOG" 2>&1 &
  BRIDGE_PID=$!

  # Monitor: restart base.launch if it crashes; exit only when bridge dies
  while true; do
    # If bridge exits, the ROS1 stack is dead — exit the container script
    if ! kill -0 "$BRIDGE_PID" >/dev/null 2>&1; then
      echo "ROS1 bridge exited, shutting down ROS1 stack" >&2
      break
    fi
    # If base hardware exits unexpectedly, restart it
    if ! kill -0 "$BASE_PID" >/dev/null 2>&1; then
      echo "base.launch exited, restarting..." >&2
      roslaunch yahboomcar_nav base.launch >"$LOG_DIR/base.launch.log" 2>&1 &
      BASE_PID=$!
    fi
    # If mode_manager exits unexpectedly, restart it
    if ! kill -0 "$MODE_PID" >/dev/null 2>&1; then
      echo "mode_manager.launch exited, restarting..." >&2
      roslaunch yahboomcar_nav mode_manager.launch &
      MODE_PID=$!
    fi
    sleep 5
  done

  # Wait for remaining processes before exiting
  wait "$BRIDGE_PID" "$MODE_PID" "$BASE_PID" 2>/dev/null || true
'
ROS1_PID=$!

wait
