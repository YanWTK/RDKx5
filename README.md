# Xiaogua: An RDK X5-Based Edge-Cloud Collaborative Multimodal Embodied Intelligent Home Service Robot

Xiaogua is an edge-cloud collaborative multimodal embodied home service robot based on the RDK X5 edge computing platform. It uses Horizon Robotics TROS together with ROS middleware for modular perception, communication, and task execution. The system integrates mobile navigation, voice interaction, object memory, vision-language target selection, visual tracking, visual servo alignment, and robotic arm manipulation to complete home service tasks such as object fetching, relative placement, patrol, remote monitoring, and app-assisted control.

This repository is organized as a cleaned open-source version of the project. Sensitive deployment details, private credentials, runtime logs, and local machine paths are intentionally excluded.

## Repository Status

This repository is intended as a research and engineering reference for a home service robot system. It contains the main software modules, launch files, configuration examples, and documentation structure used by the project. Hardware-specific parameters, model weights, private cloud credentials, and household runtime memory are not included.

The code is organized for readability and reuse. Before running it on a physical robot, review the device names, topic names, model paths, map files, navigation parameters, arm calibration, and network settings for your own platform.

## Contents

- [Highlights](#highlights)
- [System Architecture](#system-architecture)
- [Repository Layout](#recommended-repository-layout)
- [Core Modules](#core-modules)
- [Important Code Index](#important-code-index)
- [Main Workflows](#main-workflows)
- [Algorithms](#algorithms)
- [Configuration](#configuration)
- [Models](#models)
- [Hardware Reference](#hardware-reference)


## Highlights

- One-tap app startup for base capabilities: chassis control, mapping, navigation, patrol, camera, object detection, video streaming, and remote bridge access.
- AI-driven voice task service: natural-language requests are handled by an LLM task orchestrator that dynamically composes object memory, VLM visual confirmation, tracking, visual servoing, arm grasping, and delivery feedback.
- YOLO-VLM-Tracking pipeline with selected-target tracking, target loss recovery, HSV appearance matching, class constraints, velocity prediction, and buffered re-selection.
- Object memory query with approximate semantic matching, allowing queries such as "capybara plush toy" to match a remembered "plush toy" when no exact match exists.
- LLM task orchestration for complex multi-step home-service commands, including object fetching, relative placement, person-oriented delivery, and similar tasks by decomposing natural language into executable navigation, visual confirmation, tracking, grasping, placement, and delivery steps.
- App capabilities including mapping, navigation, patrol, remote camera, battery status, TTS style switching, and remote voice control.

## Capability Matrix

| Capability | Description | Main Modules |
| --- | --- | --- |
| App base stack | Start chassis, camera, YOLO, navigation, mapping, patrol, video stream, and app bridge | `app_bridge`, `navigation`, `vision_to_3d` |
| Voice task service | Understand spoken commands and invoke memory, vision, navigation, grasping, and delivery capabilities as needed | `voice_interaction`, `task_understanding`, `object_memory`, `task_orchestrator` |
| Relative placement | Place a held or fetched object to the left, right, front, or back of a reference object | `vlm_target_selector`, `object_tracker`, `arm_control` |
| Person delivery | Search for a described person, verify with VLM, record 3D position, and navigate to a safe distance | `vision_to_3d`, `task_orchestrator`, `navigation` |
| Patrol memory | Scan patrol points and build a semantic memory of visible objects | `object_memory`, `vlm_target_selector`, `navigation` |

## System Architecture

The system uses Horizon Robotics TROS on RDK X5 together with ROS middleware for modular robot software deployment. Functional modules are decoupled through topics, services, and actions. The project can be described in five layers:

1. Device and runtime layer: RDK X5, Horizon Robotics TROS runtime, microphone array, RGB-D camera, LiDAR, STM32 chassis controller, mecanum wheel base, robotic arm, TOF sensors, display, and power regulation.
2. Perception layer: camera capture, YOLO detection, depth projection, VLM target selection, object tracking, and visual overlays.
3. Voice interaction layer: wake word detection, ASR, DOA, TTS, persona control, and audio device management.
4. Task intelligence layer: LLM task understanding, object memory query, VLM target confirmation, task planning, and context memory.
5. Execution layer: navigation, mapping, visual servo alignment, grasping, placing, release control, and app bridge services.

At runtime, the system follows an event-driven design. Voice, app, and perception modules publish state or request messages. The task orchestrator subscribes to task outputs and calls downstream services only when the current state requires them. This keeps expensive vision and model calls gated to the moments when they are needed, reducing compute load on the edge device.

```text
User/App Command
  -> Voice or App Bridge
  -> LLM Task Understanding
  -> Object Memory / VLM Confirmation
  -> Navigation and Perception
  -> Tracking and Visual Servo Alignment
  -> Arm Control / Delivery / Placement
  -> TTS and App State Feedback
```

## Recommended Repository Layout

```text
xiaogua-home-service-robot/
├── README.md                         # English project overview
├── README_cn.md                      # Chinese project overview
├── docs/                             # Technical reports, architecture notes, and setup guides
│   ├── technical_report.md           # Detailed technical report
│   ├── architecture.md               # System architecture and module relationships
│   └── quick_start.md                # Quick start and deployment guide
├── config/                           # Public example configurations
│   ├── navigation/
│   │   └── patrol_points.json        # Example patrol-point configuration
│   └── voice/
│       └── voice_persona.json        # Example TTS persona configuration
├── scripts/                          # Common startup and environment scripts
│   ├── start_app.sh                  # Start the app base stack
│   ├── start_voice_fetch.sh          # Start the voice task service
│   └── setup_env.sh                  # Initialize environment variables and workspace
├── src/                              # Core source code
│   ├── app_bridge/                   # App communication, state sync, video streaming, and service bridge
│   ├── voice_interaction/            # Wake word, ASR, TTS, DOA, and voice persona control
│   ├── vlm_target_selector/          # LLM parsing, object memory, VLM target selection, and loss recovery
│   ├── object_tracker/               # Selected-target tracking and loss buffering
│   ├── vision_to_3d/                 # RGB-D based 3D target projection
│   ├── arm_control/                  # Arm grasping, release, retract, and visual servo alignment
│   └── task_orchestrator/            # High-level state machines for fetch, placement, and person delivery
└── tools/
    └── debug/
        └── doa_visualizer/           # DOA visualization and debugging tool
```

## Core Modules

| Module | Description |
| --- | --- |
| `app_bridge` | App bridge, command forwarding, MJPEG streaming, mapping/navigation service wrappers |
| `voice_interaction` | Wake word, ASR, TTS, DOA, voice bridge, persona control |
| `task_understanding` | LLM-based intent recognition and structured task generation |
| `object_memory` | Object memory query, approximate semantic matching, patrol memory access |
| `vlm_target_selector` | VLM-based target selection from numbered YOLO detections |
| `object_tracker` | Selected-object tracking and loss handling |
| `vision_to_3d` | RGB-D based 2D-to-3D projection for objects and people |
| `navigation` | Mapping, localization, global planning, local planning, and patrol points |
| `arm_control` | Robotic arm services for grasp, release, retract, and standby poses |
| `task_orchestrator` | High-level task state machine for fetching, placement, and person delivery |

## Important Code Index

| File | Function |
| --- | --- |
| `scripts/start_app.sh` | Starts the base app stack, including chassis, navigation, camera, detection, streaming, and app bridge services. |
| `scripts/start_voice_fetch.sh` | Starts the voice task service on top of the base app stack. |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/bridge_node.py` | Main app bridge node for app commands, robot status, and service forwarding. |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/mjpeg_server.py` | MJPEG video stream server used by the app and browser preview. |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/mapping_service_node.py` | Wraps mapping, localization, navigation, and patrol control as app-facing services. |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/robot_cloud_bridge.py` | Cloud/MQTT bridge with credentials removed for open-source release. |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/node.py` | XVF3800 microphone array node for audio device status, DOA, and voice hardware control. |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/wake_word_node.py` | Wake-word entry for spoken interaction. |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/asr_client.py` | ASR client wrapper for speech recognition requests. |
| `src/voice_interaction/asr_bridge/asr_ros1_bridge/tts_host_node.py` | TTS playback host and speech feedback node. |
| `src/voice_interaction/asr_bridge/asr_ros1_bridge/persona_control_node.py` | TTS style/persona switching for app and voice feedback. |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/task_understanding_node.py` | LLM task parser that converts natural language into structured intent, task fields, and executable plan steps. |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/memory_query_node.py` | Object memory query node with approximate semantic matching for imperfect object names. |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/selector_node.py` | VLM selector that chooses the target from numbered YOLO candidate boxes. |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/target_confirm_node.py` | VLM confirmation node used before execution on ambiguous visual targets. |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/lost_reselector_node.py` | Target loss recovery node that re-runs VLM selection after tracking loss. |
| `src/vlm_target_selector/yolo_detector/yolo_detector/detector_node.py` | YOLO detection node for camera frames. |
| `src/vlm_target_selector/yolo_detector/yolo_detector/yolo_engine.py` | Model inference wrapper for the YOLO detector. |
| `src/object_tracker/object_tracker/object_tracker/tracker_node.py` | Selected-object tracker with association, appearance matching, prediction, and loss buffering. |
| `src/vision_to_3d/vision_to_3d_local/vision_to_3d_local/vision_to_3d_local_node.py` | Projects 2D detections and depth into 3D object/person positions. |
| `src/vision_to_3d/vision_tf_bridge/vision_ros1_tf_bridge/selected_detection_bridge_node.py` | Sends only the selected target detection to the execution side for alignment and grasping. |
| `src/arm_control/auto_aim.py` | Segmented PID visual servo alignment before grasping or placement. |
| `src/arm_control/arm_node.py` | Robotic arm service node for grasp, release, retract, and standby actions. |
| `src/task_orchestrator/voice_fetch_orchestrator.py` | High-level task state machine for fetch, relative placement, return-to-speaker, and person delivery. |
| `src/task_orchestrator/voice_person_follow_flow.py` | Speaker/person localization and navigation flow for commands such as "come here". |

## Main Workflows

The project provides two main startup paths. In deployment, the app base stack is usually started first, and the voice task service is started only when required.

### App Startup

The app startup stack enables the robot's base capabilities:

```text
chassis -> navigation -> mapping/patrol -> camera -> YOLO -> video stream -> app bridge
```

Example command:

```bash
source scripts/setup_env.sh
bash scripts/start_app.sh
```

### Voice Task Service

The voice task service is not implemented as a fixed step-by-step pipeline. Speech is only one entry point; the AI task orchestrator dynamically composes capabilities according to the user's intent, object memory, visual feedback, and the robot's current state. A task may call wake/ASR, LLM understanding, memory lookup, navigation, VLM confirmation, target tracking, visual servoing, grasping, return, or delivery as needed, and it can re-evaluate when the target is uncertain, lost, or the scene changes.

```text
voice/app command -> AI task orchestrator
  -> on-demand use: memory / navigation / VLM / tracking / alignment / grasp / delivery
  -> continuous replanning and feedback from execution results
```

Example command:

```bash
source scripts/setup_env.sh
bash scripts/start_voice_fetch.sh
```

### Example Commands

The task understanding module is designed to preserve important semantic modifiers instead of reducing a command to only the main object class.

| User command | Expected route |
| --- | --- |
| "Bring me the water from the bedroom" | Record speaker position, find water in bedroom, grasp, return to speaker |
| "Put the plush toy to the right of the vitamin bottle in the living room" | Find and grasp plush toy, navigate to living room, find reference object, align right side, release |
| "Come here" | Record speaker direction and person pose, navigate to speaker |
| "Where is the capybara plush toy?" | Query object memory; allow approximate matching if only a related plush toy is remembered |
| "Give the bedroom water to the person mopping the floor" | Find and grasp water, search for described person, verify with VLM, navigate to standoff |

## Algorithms

### Navigation and Mapping

- Cartographer-based mapping
- Occupancy Grid map representation
- Map-based localization
- Global Costmap and Local Costmap
- Dijkstra global planning
- DWA local planning
- Navigation state machine and patrol point management

### YOLO-VLM-Tracking

The perception pipeline combines object detection, VLM target reasoning, and selected-object tracking:

- YOLO object detection
- VLM selection from numbered candidate boxes
- Selected-only target bridge
- ByteTrack-style two-stage association
- HSV appearance matching
- Class constraints
- Velocity prediction
- Lost-target buffer
- VLM re-selection when target loss persists

### Voice and Task Understanding

- Wake word detection
- ASR command transcription
- DOA-based speaker localization
- TTS feedback
- LLM structured task parsing
- Memory-aware object lookup
- Approximate memory matching for semantically related objects

### LLM Task Plan Orchestration

The task intelligence module does not only classify user intent. It converts natural language commands into a structured task JSON that contains both high-level intent fields and a dynamically generated semantic execution `plan` for the current task.

Typical output fields include:

- `intent`: task type, such as `fetch_to_speaker`, `transfer_object`, `deliver_to_person`, `come_to_speaker`, `navigate_to`, `task_chain`, or `chat`
- `target_name`: the full object description, including spatial modifiers such as "the bottle next to the remote"
- `semantic_hint`: object category and detection hints, such as `bottle`, `cup`, `book`, or `teddy bear`
- `source_location` and `destination_location`: explicit source and target locations
- `delivery_target`: speaker, location, or described person
- `placement_reference` and `placement_side`: reference object and relative placement side
- `tasks`: a compact compatibility sequence for the current executor
- `plan`: a richer action sequence for task orchestration

The `plan` field represents the task as executable semantic steps:

```json
[
  {"action": "find_object", "location": "bedroom", "target": "water", "search_locations": ["bedroom"]},
  {"action": "grasp_object", "location": "bedroom", "target": "water"},
  {"action": "find_person", "target": "person mopping the floor"},
  {"action": "navigate_to", "location": "person", "target": "person mopping the floor"}
]
```

The orchestrator then maps these semantic steps to robot capabilities:

- `find_object`: query object memory, navigate to the remembered point, or fall back to candidate search locations
- `grasp_object`: confirm the object with VLM, select the target, track it, align with visual servoing, and grasp
- `navigate_to`: send a navigation goal to a named location or recorded person pose
- `place_relative`: find the reference object and align the held object to the requested side
- `find_person`: search for a described person, verify the person with VLM, record the 3D pose, and navigate to a safe standoff
- `return_to_speaker`: navigate back to the previously recorded speaker position

This design allows the same language interface to handle simple navigation, object fetching, relative placement, memory queries, and person-oriented delivery without hard-coding every sentence pattern.

## Configuration

Most deployment-specific values should be configured through environment variables or launch arguments rather than hard-coded paths.

| Variable | Purpose | Example |
| --- | --- | --- |
| `ROBOT_IP` | Robot address used by external clients | `robot-ip.example` |
| `YOLO_MODEL_PATH` | Local object detection model path | `/opt/xiaogua/models/yolo_model.bin` |
| `YOLO_CLASS_NAMES` | Detection class list | Common household object classes |
| `DASHSCOPE_API_KEY` | Cloud model API key, if cloud models are used | `your_api_key_here` |
| `VLM_URL` | Local VLM endpoint, if local VLM is used | `http://127.0.0.1:8000/analyze` |
| `LLM_URL` | Local LLM endpoint, if local LLM is used | `http://127.0.0.1:8000/analyze` |
| `MQTT_HOST` | Optional cloud message broker host | `your_mqtt_host` |

Recommended setup:

```bash
source scripts/setup_env.sh
```

Then edit the values in `scripts/setup_env.sh` or export your own values before launching the stack.

## Safety and Runtime Boundaries

This project controls a mobile base and a robotic arm. Before running on hardware:

- Verify emergency stop behavior and manual override.
- Test navigation in a clear area before enabling full autonomy.
- Calibrate arm poses, camera extrinsics, and grasp distance on the target robot.
- Keep people away from the arm workspace during grasp and release tests.
- Do not expose rosbridge, MJPEG, MQTT, or command topics directly to the public internet without authentication and network isolation.
- Keep API keys and cloud credentials outside the repository.

## Models

The project can be configured with cloud or local model providers. The reference configuration uses:

- Task understanding, memory reasoning, and speech generation: `qwen3.6-flash`
- Vision-language target selection: `qwen3-vl-plus`
- TTS: `qwen3-tts-instruct-flash-realtime`
- ASR: `qwen3-asr-flash`

Model names are configurable and should be replaced as needed for open deployment.

## Hardware Reference

- RDK X5 edge computing platform
- Horizon Robotics TROS robot software framework on RDK X5
- XVF3800 microphone array
- Astra Plus Pro RGB-D camera
- STM32 chassis controller
- 4ROS LiDAR
- TOF sensors
- Six-DOF robotic arm
- Mecanum wheel chassis
- LED display
- DC-DC regulated power module

## Documentation

Additional documents can be placed under `docs/`:

- `docs/technical_report.md`: project report and system design details
- `docs/architecture.md`: module-level architecture
- `docs/quick_start.md`: startup notes and local configuration checklist

## Roadmap

Planned directions for continued development:

- Cleaner hardware abstraction for different chassis and arm platforms
- More robust sound source localization in reflective home environments
- Person attribute and activity recognition for delivery tasks
- Better simulation support for navigation and manipulation testing
- Dataset and benchmark scripts for YOLO-VLM-Tracking evaluation
- Containerized deployment examples

## Open-Source Notes

Before publishing, remove or replace:

- API keys and tokens
- FRP, MQTT, and remote server credentials
- Private IP addresses and domain names
- Personal object memory data
- Runtime logs
- Build artifacts: `build/`, `install/`, `log/`
- Python caches: `__pycache__/`
- Model weights without redistribution permission

Recommended placeholders:

```bash
export DASHSCOPE_API_KEY="your_api_key_here"
export ROBOT_IP="robot-ip.example"
export MQTT_HOST="your_mqtt_host"
```

## Citation

If you use this project in a report, paper, or derivative robot system, cite the repository and describe the hardware and model configuration used in your experiment.

```bibtex
@misc{xiaogua_home_service_robot,
  title = {Xiaogua Home Service Robot},
  author = {Xiaogua Robot Project Contributors},
  year = {2026},
  howpublished = {Open-source robotics project}
}
```

## License

Add a license before publishing. For academic and community reuse, Apache-2.0 or MIT are common choices. For stricter control over commercial use, choose a more restrictive license after reviewing the dependencies and model licenses.
