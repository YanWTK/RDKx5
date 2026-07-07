# Architecture

The system is organized as modular ROS middleware components. Modules communicate through topics, services, and actions.

## Layers

1. Device layer: sensors, chassis, arm, display, and power modules.
2. Perception layer: camera, YOLO, depth projection, VLM target selection, and tracking.
3. Voice layer: wake word, ASR, DOA, TTS, and voice bridge.
4. Task layer: LLM task understanding, object memory, VLM confirmation, and orchestration.
5. Execution layer: navigation, visual servo alignment, grasping, placement, and app bridge.

## Main Workflows

- App base stack: chassis, navigation, camera, YOLO, video stream, and app bridge.
- Voice fetch stack: voice interaction, task understanding, memory query, target confirmation, tracking, alignment, grasping, and delivery.
