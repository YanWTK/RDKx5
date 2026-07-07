#!/usr/bin/env bash
set -euo pipefail

# Source this file before running the demo scripts.
# Replace these placeholders with your local deployment settings.

export ROBOT_IP="${ROBOT_IP:-robot-ip.example}"
export DASHSCOPE_API_KEY="${DASHSCOPE_API_KEY:-your_api_key_here}"
export YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-/path/to/yolo_model.bin}"
export YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box}"

export LLM_MODEL="${LLM_MODEL:-qwen3.6-flash}"
export VLM_MODEL="${VLM_MODEL:-qwen3-vl-plus}"
export ASR_MODEL="${ASR_MODEL:-qwen3-asr-flash}"
export TTS_MODEL="${TTS_MODEL:-qwen3-tts-instruct-flash-realtime}"

echo "Environment placeholders loaded. Review scripts/ and config/ before running on hardware."

