# Quick Start

This open-source layout is a cleaned project snapshot. Before running on a robot, review and adapt all paths, device names, model paths, network settings, and credentials.

## Main Entry Points

```bash
bash scripts/start_app.sh
bash scripts/start_voice_fetch.sh
```

## Required Local Configuration

- Camera device and image topics
- Microphone array device
- Chassis and arm control services
- Navigation map and patrol points
- YOLO model path and class names
- LLM/VLM/ASR/TTS provider configuration
- App, MQTT, FRP, and video streaming settings if used

Do not commit private keys, tokens, server addresses, or personal memory data.
