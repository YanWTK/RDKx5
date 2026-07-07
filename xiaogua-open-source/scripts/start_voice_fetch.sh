#!/usr/bin/env bash
set -eo pipefail

VOICE_LOCK_FILE="${VOICE_LOCK_FILE:-/tmp/robopilot_voice_fetch_autoaim.lock}"
exec 9>"$VOICE_LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: Voice auto-aim companion stack is already running (lock: $VOICE_LOCK_FILE)." >&2
  exit 1
fi

# App companion mode:
# Run start_robopilot_app.sh first, then run this script to add voice fetch +
# VLM target selection + tracking + ROS1 auto_aim + arm grasp/return. This script deliberately
# does not start/stop base, navigation, camera, App MJPEG, App cmd_vel relay,
# or the App ROS2 rosbridge.

ROS2_WS="${ROS2_WS:-/opt/xiaogua/ros2_ws}"
ROS1_CONTAINER="${ROS1_CONTAINER:-xiaogua_runtime}"
ROS1_BRIDGE_PORT="${ROS1_BRIDGE_PORT:-9091}"
ROS1_BRIDGE_URL="${ROS1_BRIDGE_URL:-ws://127.0.0.1:${ROS1_BRIDGE_PORT}}"
LOG_DIR="${LOG_DIR:-/tmp/voice_fetch_autoaim_logs}"
HOST_VOICE_PERSONA_PATH="${HOST_VOICE_PERSONA_PATH:-/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json}"
ROS1_VOICE_PERSONA_PATH="${ROS1_VOICE_PERSONA_PATH:-/opt/xiaogua/runtime_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json}"
VOICE_PERSONA_PROFILE="${VOICE_PERSONA_PROFILE:-}"

CAPTURE_COUNT="${CAPTURE_COUNT:-3}"
CAPTURE_INTERVAL_SEC="${CAPTURE_INTERVAL_SEC:-0.12}"
DOA_MIN_WINNER_RATIO="${DOA_MIN_WINNER_RATIO:-0.45}"
DOA_ACCEPT_WINDOW_DEG="${DOA_ACCEPT_WINDOW_DEG:-25.0}"
DOA_CLUSTER_WINDOW_DEG="${DOA_CLUSTER_WINDOW_DEG:-30.0}"
DOA_IGNORE_EDGE_SEC="${DOA_IGNORE_EDGE_SEC:-0.2}"
DELIVER_PERSON_BACKUP_M="${DELIVER_PERSON_BACKUP_M:-0.5}"
DELIVER_PERSON_BACKUP_SPEED="${DELIVER_PERSON_BACKUP_SPEED:-0.12}"
DELIVER_PERSON_SEARCH_STEP_DEG="${DELIVER_PERSON_SEARCH_STEP_DEG:-30.0}"
DELIVER_PERSON_SEARCH_ANGULAR_SPEED="${DELIVER_PERSON_SEARCH_ANGULAR_SPEED:-0.18}"
DELIVER_PERSON_SEARCH_MAX_DEG="${DELIVER_PERSON_SEARCH_MAX_DEG:-360.0}"
GRASP_PIXEL_X="${GRASP_PIXEL_X:-320}"
GRASP_TOL_PX="${GRASP_TOL_PX:-10}"
PLACE_LEFT_PIXEL_X="${PLACE_LEFT_PIXEL_X:-480}"
PLACE_RIGHT_PIXEL_X="${PLACE_RIGHT_PIXEL_X:-160}"
PLACE_TOL_PX="${PLACE_TOL_PX:-20}"
PLACE_TARGET_DIST_CM="${PLACE_TARGET_DIST_CM:-20.0}"
PLACE_STAGE1_DIST_CM="${PLACE_STAGE1_DIST_CM:-24.0}"
PLACE_TOL_DIST_STAGE1="${PLACE_TOL_DIST_STAGE1:-2.5}"
PLACE_TOL_DIST_FINAL="${PLACE_TOL_DIST_FINAL:-2.0}"
PLACE_TOL_YAW="${PLACE_TOL_YAW:-6.0}"
PLACE_TOL_PIXEL_ROUGH="${PLACE_TOL_PIXEL_ROUGH:-40}"
PLACE_MIN_VEL_Y="${PLACE_MIN_VEL_Y:-0.04}"
PLACE_MIN_VEL_Z="${PLACE_MIN_VEL_Z:-0.08}"
PLACEMENT_PRE_ALIGN_SETTLE_SEC="${PLACEMENT_PRE_ALIGN_SETTLE_SEC:-1.5}"
PERSON_RETRY_ENABLED="${PERSON_RETRY_ENABLED:-true}"
PERSON_RETRY_LISTEN_TIMEOUT_SEC="${PERSON_RETRY_LISTEN_TIMEOUT_SEC:-6.0}"
PERSON_RETRY_MIN_LISTEN_SEC="${PERSON_RETRY_MIN_LISTEN_SEC:-1.2}"
PERSON_RETRY_PROMPT="${PERSON_RETRY_PROMPT:-我没看到你，你再说一句我在这里}"
USE_LOCAL_LLM="${USE_LOCAL_LLM:-false}"
LLM_URL="${LLM_URL:-http://127.0.0.1:8000/analyze}"
TASK_MODEL="${TASK_MODEL:-qwen3.6-flash}"
ENABLE_LLM_SPEECH="${ENABLE_LLM_SPEECH:-true}"
LLM_SPEECH_MODEL="${LLM_SPEECH_MODEL:-$TASK_MODEL}"
LLM_SPEECH_BASE_URL="${LLM_SPEECH_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
LLM_SPEECH_TIMEOUT_SEC="${LLM_SPEECH_TIMEOUT_SEC:-2.5}"
LLM_SPEECH_TEMPERATURE="${LLM_SPEECH_TEMPERATURE:-0.95}"
USE_LOCAL_VLM="${USE_LOCAL_VLM:-false}"
VLM_URL="${VLM_URL:-http://127.0.0.1:8000/analyze}"
BAILIAN_MODEL="${BAILIAN_MODEL:-qwen3-vl-plus}"
START_WAKE="${START_WAKE:-true}"
XVF3800_REENUMERATE_ON_START="${XVF3800_REENUMERATE_ON_START:-true}"
AUTO_CONFIGURE_HOST_LOGS="${AUTO_CONFIGURE_HOST_LOGS:-true}"
ENABLE_CPU_AFFINITY="${ENABLE_CPU_AFFINITY:-true}"
NAV_CPUSET="${NAV_CPUSET:-0,1}"
VOICE_CPUSET="${VOICE_CPUSET:-2,3}"
VISION_CPUSET="${VISION_CPUSET:-3-7}"
ASTRA_COLOR_WIDTH="${ASTRA_COLOR_WIDTH:-640}"
ASTRA_COLOR_HEIGHT="${ASTRA_COLOR_HEIGHT:-480}"
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-/opt/xiaogua/models/yolo_model.bin}"
YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-}"
YOLO_CONF_THRESHOLD="${YOLO_CONF_THRESHOLD:-0.07}"
YOLO_PREPROCESS_MODE="${YOLO_PREPROCESS_MODE:-letterbox}"
if [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_alt.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_scene.bin" ] ||
   [ "$YOLO_MODEL_PATH" = "/opt/xiaogua/models/yolo_model_good.bin" ]; then
  YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box}"
fi

if [ "$ENABLE_CPU_AFFINITY" = "true" ] && { ! command -v taskset >/dev/null 2>&1 || [ "$(nproc)" -lt 8 ]; }; then
  echo "WARN: CPU affinity disabled (taskset missing or fewer than 8 CPUs)."
  ENABLE_CPU_AFFINITY=false
fi

mkdir -p "$LOG_DIR"

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
  USB_AUTOSUSPEND_SCRIPT="$ROS2_WS/src/robopilot_app_bridge/scripts/disable_usb_autosuspend.sh"
  if [ -x "$USB_AUTOSUSPEND_SCRIPT" ]; then
    "$USB_AUTOSUSPEND_SCRIPT" || true
  elif [ -f "$USB_AUTOSUSPEND_SCRIPT" ]; then
    bash "$USB_AUTOSUSPEND_SCRIPT" || true
  else
    echo "WARN: USB autosuspend script not found: $USB_AUTOSUSPEND_SCRIPT"
  fi
fi

configure_xvf3800_playback_volume

if [ "$AUTO_CONFIGURE_HOST_LOGS" = "true" ] && [ "${EUID}" -eq 0 ]; then
  CLEAN_OLD_LOGS=false \
    "$ROS2_WS/src/robopilot_app_bridge/scripts/install_host_log_limits.sh"
fi

source /opt/ros/humble/setup.bash
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
fi
source "$ROS2_WS/install/setup.bash"

# Selective colcon builds can leave a newly installed package out of the
# workspace-level setup file. Register the reSpeaker prefix explicitly so the
# wake launch is always discoverable.
RESPEAKER_PREFIX="$ROS2_WS/install/respeaker_xvf3800_ros2"
RESPEAKER_SETUP="$RESPEAKER_PREFIX/share/respeaker_xvf3800_ros2/package.bash"
if [ ! -f "$RESPEAKER_SETUP" ]; then
  echo "ERROR: reSpeaker package is not installed: $RESPEAKER_SETUP" >&2
  echo "ERROR: build it with: colcon build --packages-select respeaker_xvf3800_ros2" >&2
  exit 1
fi
export AMENT_PREFIX_PATH="$RESPEAKER_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export COLCON_PREFIX_PATH="$RESPEAKER_PREFIX${COLCON_PREFIX_PATH:+:$COLCON_PREFIX_PATH}"
source "$RESPEAKER_SETUP"
if ! ros2 pkg prefix respeaker_xvf3800_ros2 >/dev/null 2>&1; then
  echo "ERROR: failed to register ROS2 package respeaker_xvf3800_ros2" >&2
  exit 1
fi

export VOICE_PERSONA_PATH="$HOST_VOICE_PERSONA_PATH"
if [ -n "$VOICE_PERSONA_PROFILE" ]; then
  export VOICE_PERSONA_PROFILE
fi

if ! pgrep -af "start_robopilot_app.sh|robopilot_app_bridge.mjpeg_server|robopilot_app_bridge.cmd_vel_relay|robopilot_app_bridge.mapping_service_node" >/dev/null; then
  echo "WARN: 未检测到 App 一键启动进程。这个脚本只补充完整语音取物流程，不启动底盘/导航/相机。"
  echo "WARN: 请先运行: cd /opt/xiaogua/ros2_ws && bash src/robopilot_app_bridge/scripts/start_robopilot_app.sh"
fi

if { [ "$USE_LOCAL_LLM" != "true" ] || [ "$USE_LOCAL_VLM" != "true" ]; } && [ -z "${DASHSCOPE_API_KEY:-}" ]; then
  echo "WARN: 当前配置使用百炼，但 DASHSCOPE_API_KEY 为空，语言理解/视觉确认可能失败。"
fi

cleanup_ros1_voice_nodes() {
  # The App stack shares this container and ROS master. Never signal the
  # container or broadly kill ROS processes here; doing so also takes down
  # base.launch, mode_manager and robopilot_ros1_app_bridge.py.
  docker exec "$ROS1_CONTAINER" bash -lc '
    bridge_port="$1"
    source /opt/ros/noetic/setup.bash
    rosnode kill /voice_fetch_orchestrator /auto_aim /arm_grasp_service_node /rosbridge_websocket /rosapi 2>/dev/null || true
    pkill -TERM -f "[v]oice_fetch_orchestrator.py" 2>/dev/null || true
    pkill -TERM -f "[a]uto_aim.py" 2>/dev/null || true
    pkill -TERM -f "[a]rm_node.py" 2>/dev/null || true
    pkill -TERM -f "[r]osbridge_websocket.launch port:=${bridge_port}" 2>/dev/null || true
  ' _ "$ROS1_BRIDGE_PORT" >/dev/null 2>&1 || true
}

cleanup_local_voice_nodes() {
  local patterns=(
    "[r]os2 launch vlm_target_selector vlm_target_selector.launch.py"
    "[v]lm_target_selector_node"
    "[r]os2 launch object_tracker object_tracker.launch.py"
    "[o]bject_tracker_node"
    "[r]os2 launch vlm_target_selector lost_reselector.launch.py"
    "[l]ost_reselector_node"
    "[r]os2 launch vlm_target_selector fetch_task_support.launch.py"
    "[t]ask_understanding_node"
    "[o]bject_memory_query_node"
    "[t]arget_confirm_node"
    "[m]emory_target_select_adapter_node"
    "[f]etch_task_bridge_node"
    "[s]elected_detection_bridge_node"
    "[v]ision_to_3d_local_node"
    "[v]ision_tf_bridge_node"
    "[r]os2 launch respeaker_xvf3800_ros2 respeaker_xvf3800_wake.launch.py"
    "[r]os2 launch respeaker_xvf3800_ros2 respeaker_xvf3800_asr.launch.py"
    "[r]espeaker_xvf3800_node"
    "[r]espeaker_xvf3800_asr_node"
    "[r]espeaker_xvf3800_wake_node"
    "[r]os2 run asr_ros1_bridge bridge_node"
    "[a]sr_to_ros1_bridge"
    "[r]os2 run asr_ros1_bridge persona_control_node"
    "[v]oice_persona_control"
    "[r]os2 run asr_ros1_bridge tts_host_node"
    "[t]ts_host_node"
    "[r]os2 launch doa_ros1_bridge doa_bridge.launch.py"
    "[d]oa_to_ros1_bridge"
  )
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" || true
  done
  # Wait for processes to exit gracefully (up to 5s)
  local _i _pattern
  for _i in 1 2 3 4 5; do
    sleep 1
    local _any_alive=false
    for _pattern in "${patterns[@]}"; do
      if pgrep -f "$_pattern" >/dev/null 2>&1; then
        _any_alive=true
        break
      fi
    done
    if [ "$_any_alive" = false ]; then
      return 0
    fi
  done
  for _pattern in "${patterns[@]}"; do
    pkill -KILL -f "$_pattern" || true
  done
}

cleanup() {
  trap - EXIT INT TERM
  cleanup_ros1_voice_nodes
  cleanup_local_voice_nodes

  # Kill all descendant processes (children + grandchildren)
  local pids
  pids="$(jobs -pr)" || true
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

start_bg() {
  local name="$1"
  shift
  echo "START $name -> $LOG_DIR/$name.log"
  (trap '' INT; exec "$@" >"$LOG_DIR/$name.log" 2>&1) &
  sleep 0.5
}

start_bg_on_cpus() {
  local name="$1"
  local cpus="$2"
  shift 2
  if [ "$ENABLE_CPU_AFFINITY" = "true" ]; then
    start_bg "$name" taskset -c "$cpus" "$@"
  else
    start_bg "$name" "$@"
  fi
}

mirror_ros1_container_log() {
  local filename="$1"
  echo "MIRROR ROS1 $filename -> $LOG_DIR/$filename"
  (
    trap '' INT
    exec docker exec "$ROS1_CONTAINER" bash -lc "
      mkdir -p '$LOG_DIR'
      touch '$LOG_DIR/$filename'
      tail -n +1 -F '$LOG_DIR/$filename'
    " >"$LOG_DIR/$filename" 2>"$LOG_DIR/${filename%.log}_mirror.log"
  ) &
  sleep 0.2
}

wait_for_ros1_bridge() {
  local deadline=$((SECONDS + 45))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ss -ltn 2>/dev/null | grep -q ":${ROS1_BRIDGE_PORT} "; then
      return 0
    fi
    sleep 1
  done
  echo "WARN: 宿主机未检测到 ROS1 rosbridge ${ROS1_BRIDGE_PORT} 监听，继续启动；桥接节点会自动重连。查看 $LOG_DIR/ros1_rosbridge.log"
  return 0
}

echo "Logs: $LOG_DIR"
echo "Mode: App companion, full voice fetch (auto_aim + arm + return)"
echo "ROS1 voice/fetch bridge: $ROS1_BRIDGE_URL"
echo "Using gated vision camera: /camera/color/image_raw"
echo "Using App YOLO: /yolo_detector/detections"
echo "Target confirm YOLO: model=$YOLO_MODEL_PATH conf=$YOLO_CONF_THRESHOLD preprocess=$YOLO_PREPROCESS_MODE"
echo "LLM mode: $([ "$USE_LOCAL_LLM" = "true" ] && echo local || echo bailian), model=$TASK_MODEL"
echo "LLM speech: $ENABLE_LLM_SPEECH, model=$LLM_SPEECH_MODEL"
echo "VLM mode: $([ "$USE_LOCAL_VLM" = "true" ] && echo local || echo bailian), model=$BAILIAN_MODEL"
echo "Voice persona: ${VOICE_PERSONA_PROFILE:-active_profile in $HOST_VOICE_PERSONA_PATH}"
echo "Auto-aim tuning: grasp=${GRASP_PIXEL_X}px tol=${GRASP_TOL_PX}px, place_left=${PLACE_LEFT_PIXEL_X}px, place_right=${PLACE_RIGHT_PIXEL_X}px, place_tol=${PLACE_TOL_PX}px"
echo "Place auto-aim motion: target=${PLACE_TARGET_DIST_CM}cm stage1=${PLACE_STAGE1_DIST_CM}cm tol_final=${PLACE_TOL_DIST_FINAL}cm yaw_tol=${PLACE_TOL_YAW}deg"

cleanup_local_voice_nodes
if [ "$XVF3800_REENUMERATE_ON_START" = "true" ]; then
  XVF3800_REENUMERATE_SCRIPT="$ROS2_WS/src/robopilot_app_bridge/scripts/reenumerate_xvf3800.sh"
  if [ -x "$XVF3800_REENUMERATE_SCRIPT" ]; then
    "$XVF3800_REENUMERATE_SCRIPT" 2>&1 | tee "$LOG_DIR/xvf3800_reenumerate.log"
  elif [ -f "$XVF3800_REENUMERATE_SCRIPT" ]; then
    bash "$XVF3800_REENUMERATE_SCRIPT" 2>&1 | tee "$LOG_DIR/xvf3800_reenumerate.log"
  else
    echo "WARN: XVF3800 re-enumerate script not found: $XVF3800_REENUMERATE_SCRIPT"
  fi
  configure_xvf3800_playback_volume
fi
docker start "$ROS1_CONTAINER" >/dev/null
sleep 1
cleanup_ros1_voice_nodes

ROS1_CPU_ARGS=()
if [ "$ENABLE_CPU_AFFINITY" = "true" ]; then
  ROS1_CPU_ARGS=(taskset -c "$NAV_CPUSET")
fi
(trap '' INT; exec docker exec "$ROS1_CONTAINER" "${ROS1_CPU_ARGS[@]}" bash -lc "
  set -eo pipefail
  source /opt/ros/noetic/setup.bash
  source /opt/xiaogua/runtime_ws/yahboomcar_ws/devel/setup.bash
  source /opt/xiaogua/runtime_ws/software/library_ws/devel/setup.bash --extend
  source /opt/xiaogua/runtime_ws/software/carto_ws/install_isolated/setup.bash --extend
  export ROBOT_TYPE=X3plus
  export RPLIDAR_TYPE=X3
  export DASHSCOPE_API_KEY='${DASHSCOPE_API_KEY:-}'

  mkdir -p '$LOG_DIR'

  cleanup_voice_ros1() {
    for pid in \${ORCH_PID:-} \${ARM_PID:-} \${AUTOAIM_PID:-} \${ROSBRIDGE_PID:-}; do
      if [ -n \"\$pid\" ]; then
        kill \"\$pid\" >/dev/null 2>&1 || true
      fi
    done
    wait \${ORCH_PID:-} \${ARM_PID:-} \${AUTOAIM_PID:-} \${ROSBRIDGE_PID:-} >/dev/null 2>&1 || true
  }
  trap cleanup_voice_ros1 EXIT INT TERM

  roslaunch rosbridge_server rosbridge_websocket.launch port:=$ROS1_BRIDGE_PORT >'$LOG_DIR/ros1_rosbridge.log' 2>&1 &
  ROSBRIDGE_PID=\$!

  sleep 3
  rosrun nav_pkg auto_aim.py \
    _detection_topic:=/tracked_yolov8/detections \
    _min_detection_conf:=0.05 \
    _target_pixel_x:=$GRASP_PIXEL_X \
    _tol_pixel_fine:=$GRASP_TOL_PX >'$LOG_DIR/ros1_auto_aim.log' 2>&1 &
  AUTOAIM_PID=\$!

  sleep 1
  rosrun nav_pkg arm_node.py >'$LOG_DIR/ros1_arm_node.log' 2>&1 &
  ARM_PID=\$!

  sleep 1
  roslaunch nav_pkg voice_fetch_orchestrator.launch \
    stop_after_auto_aim:=false \
    capture_count:=$CAPTURE_COUNT \
    doa_min_winner_ratio:=$DOA_MIN_WINNER_RATIO \
    doa_accept_window_deg:=$DOA_ACCEPT_WINDOW_DEG \
    doa_cluster_window_deg:=$DOA_CLUSTER_WINDOW_DEG \
    doa_ignore_edge_sec:=$DOA_IGNORE_EDGE_SEC \
    deliver_person_backup_m:=$DELIVER_PERSON_BACKUP_M \
    deliver_person_backup_speed:=$DELIVER_PERSON_BACKUP_SPEED \
    deliver_person_search_step_deg:=$DELIVER_PERSON_SEARCH_STEP_DEG \
    deliver_person_search_angular_speed:=$DELIVER_PERSON_SEARCH_ANGULAR_SPEED \
    deliver_person_search_max_deg:=$DELIVER_PERSON_SEARCH_MAX_DEG \
    task_understanding_timeout_sec:=22.0 \
    memory_query_timeout_sec:=22.0 \
    target_confirm_timeout_sec:=18.0 \
    memory_select_timeout_sec:=18.0 \
    tracker_lock_timeout_sec:=6.0 \
    align_timeout_sec:=35.0 \
    grasp_pixel_x:=$GRASP_PIXEL_X \
    grasp_tol_px:=$GRASP_TOL_PX \
    place_left_pixel_x:=$PLACE_LEFT_PIXEL_X \
    place_right_pixel_x:=$PLACE_RIGHT_PIXEL_X \
    place_tol_px:=$PLACE_TOL_PX \
    place_target_dist_cm:=$PLACE_TARGET_DIST_CM \
    place_stage1_dist_cm:=$PLACE_STAGE1_DIST_CM \
    place_tol_dist_stage1:=$PLACE_TOL_DIST_STAGE1 \
    place_tol_dist_final:=$PLACE_TOL_DIST_FINAL \
    place_tol_yaw:=$PLACE_TOL_YAW \
    place_tol_pixel_rough:=$PLACE_TOL_PIXEL_ROUGH \
    place_min_vel_y:=$PLACE_MIN_VEL_Y \
    place_min_vel_z:=$PLACE_MIN_VEL_Z \
    placement_pre_align_settle_sec:=$PLACEMENT_PRE_ALIGN_SETTLE_SEC \
    person_retry_enabled:=$PERSON_RETRY_ENABLED \
    person_retry_listen_timeout_sec:=$PERSON_RETRY_LISTEN_TIMEOUT_SEC \
    person_retry_min_listen_sec:=$PERSON_RETRY_MIN_LISTEN_SEC \
    person_retry_prompt:='$PERSON_RETRY_PROMPT' \
    use_robot_pose_if_no_person:=true \
    speech_profile_path:=$ROS1_VOICE_PERSONA_PATH \
    speech_profile:=$VOICE_PERSONA_PROFILE \
    enable_llm_speech:=$ENABLE_LLM_SPEECH \
    llm_speech_model:=$LLM_SPEECH_MODEL \
    llm_speech_base_url:=$LLM_SPEECH_BASE_URL \
    llm_speech_timeout_sec:=$LLM_SPEECH_TIMEOUT_SEC \
    llm_speech_temperature:=$LLM_SPEECH_TEMPERATURE >'$LOG_DIR/ros1_voice_fetch_orchestrator.log' 2>&1 &
  ORCH_PID=\$!
  wait \"\$ORCH_PID\"
") &
ROS1_VOICE_PID=$!

mirror_ros1_container_log ros1_rosbridge.log
mirror_ros1_container_log ros1_auto_aim.log
mirror_ros1_container_log ros1_arm_node.log
mirror_ros1_container_log ros1_voice_fetch_orchestrator.log

start_bg_on_cpus vlm_target_selector "$VISION_CPUSET" ros2 launch vlm_target_selector vlm_target_selector.launch.py \
  image_topic:=/camera/color/image_raw \
  detection_topic:=/yolo_detector/detections \
  use_local_vlm:="$USE_LOCAL_VLM" \
  vlm_url:="$VLM_URL" \
  bailian_model:="$BAILIAN_MODEL" \
  bailian_enable_thinking:=false \
  request_timeout_sec:=18.0 \
  max_data_age_sec:=1.5 \
  selection_frame_count:=5 \
  selection_timeout_sec:=0.8 \
  always_save_debug_images:=false \
  task_gated:=true

start_bg_on_cpus object_tracker "$VISION_CPUSET" ros2 launch object_tracker object_tracker.launch.py \
  image_topic:=/camera/color/image_raw \
  detection_topic:=/yolo_detector/detections \
  selected_detection_topic:=/vlm_target_selector/selected_detection \
  track_buffer:=30 \
  enable_debug_image:=false \
  max_new_tracks_per_frame:=2

start_bg_on_cpus lost_reselector "$VISION_CPUSET" ros2 launch vlm_target_selector lost_reselector.launch.py \
  target_name_topic:=/vlm_target_selector/current_target_name \
  lost_reselect_delay_sec:=1.0 \
  max_lost_no_object_sec:=8.0 \
  reselect_cooldown_sec:=5.0 \
  save_debug_images:=false \
  trigger_on_lost_final:=true

if [ "$START_WAKE" = "true" ]; then
  start_bg_on_cpus respeaker "$VOICE_CPUSET" ros2 launch respeaker_xvf3800_ros2 respeaker_xvf3800_wake.launch.py
else
  start_bg_on_cpus respeaker "$VOICE_CPUSET" ros2 launch respeaker_xvf3800_ros2 respeaker_xvf3800_asr.launch.py
fi

start_bg_on_cpus tts_host "$VOICE_CPUSET" ros2 run asr_ros1_bridge tts_host_node

start_bg_on_cpus persona_control "$VOICE_CPUSET" ros2 run asr_ros1_bridge persona_control_node

wait_for_ros1_bridge

start_bg_on_cpus asr_ros1_bridge "$VOICE_CPUSET" ros2 run asr_ros1_bridge bridge_node --ros-args \
  -p rosbridge_url:="$ROS1_BRIDGE_URL"

start_bg_on_cpus doa_ros1_bridge "$VOICE_CPUSET" ros2 launch doa_ros1_bridge doa_bridge.launch.py \
  rosbridge_url:="$ROS1_BRIDGE_URL" \
  forward_vad:=true

FETCH_TASK_SUPPORT_ARGS=(
  rosbridge_url:="$ROS1_BRIDGE_URL"
  image_topic:=/camera/color/image_raw
  depth_topic:=/camera/depth/image_raw
  camera_info_topic:=/camera/depth/camera_info
  source_image_width:="$ASTRA_COLOR_WIDTH"
  source_image_height:="$ASTRA_COLOR_HEIGHT"
  person_3d_detection_topic:=/yolo_detector/detections
  person_3d_output_topic:=/vision/target_point_local
  person_3d_target_class:=person
  yolo_model_path:="$YOLO_MODEL_PATH"
  yolo_class_names:="$YOLO_CLASS_NAMES"
  yolo_preprocess_mode:="$YOLO_PREPROCESS_MODE"
  conf_threshold:="$YOLO_CONF_THRESHOLD"
  capture_count:="$CAPTURE_COUNT"
  capture_interval_sec:="$CAPTURE_INTERVAL_SEC"
  use_local_llm:="$USE_LOCAL_LLM"
  llm_url:="$LLM_URL"
  task_model:="$TASK_MODEL"
  speech_profile_path:="$HOST_VOICE_PERSONA_PATH"
  use_local_vlm:="$USE_LOCAL_VLM"
  vlm_url:="$VLM_URL"
  bailian_model:="$BAILIAN_MODEL"
  bailian_enable_thinking:=false
  vision_task_gated:=true
)
if [ -n "$VOICE_PERSONA_PROFILE" ]; then
  FETCH_TASK_SUPPORT_ARGS+=(speech_profile:="$VOICE_PERSONA_PROFILE")
fi
start_bg_on_cpus fetch_task_support "$VISION_CPUSET" ros2 launch vlm_target_selector fetch_task_support.launch.py \
  "${FETCH_TASK_SUPPORT_ARGS[@]}"

echo
echo "Full voice fetch companion stack is running."
echo "Manual test:"
echo "  docker exec -it $ROS1_CONTAINER bash"
echo "  source /opt/ros/noetic/setup.bash && source /opt/xiaogua/runtime_ws/yahboomcar_ws/devel/setup.bash"
echo "  rostopic pub --once /voice_fetch/command std_msgs/String \"data: '我渴了，给我哪一瓶冰红茶'\""
echo
echo "State:"
echo "  docker exec -it $ROS1_CONTAINER bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo /voice_fetch/state'"
echo

wait
