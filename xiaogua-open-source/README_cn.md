# 小瓜家庭服务机器人

小瓜是一款基于 RDK X5 边缘计算平台的端云协同多模态具身智能家庭服务机器人。系统结合地平线 TROS 机器人开发框架和 ROS 中间件，完成感知、通信与任务执行模块化部署，并集成移动导航、语音交互、物品记忆、视觉语言目标选择、视觉追踪、视觉伺服对准和机械臂操作能力，可以完成家庭场景中的取物、搬运、相对放置、巡逻、远程看护和 App 控制等任务。

本仓库是项目的开源整理版本。真实部署中的密钥、私有服务器地址、运行日志、本机绝对路径和个人物品记忆数据不应进入开源仓库。

## 仓库状态

本仓库定位为家庭服务机器人系统的科研与工程参考实现，包含项目的主要软件模块、启动文件、配置示例和文档结构。硬件专用参数、模型权重、私有云端凭据和真实家庭环境中的运行记忆数据不包含在仓库中。

代码已经按开源阅读和复用做了整理。实际部署到机器人前，需要根据自己的平台检查设备名、话题名、模型路径、地图文件、导航参数、机械臂标定和网络配置。

## 目录导航

- [项目亮点](#项目亮点)
- [系统架构](#系统架构)
- [推荐开源目录结构](#推荐开源目录结构)
- [核心模块说明](#核心模块说明)
- [重要代码索引](#重要代码索引)
- [主要工作流程](#主要工作流程)
- [关键算法](#关键算法)
- [配置说明](#配置说明)
- [模型配置](#模型配置)
- [硬件参考](#硬件参考)
- [开源整理注意事项](#开源整理注意事项)
- [许可证](#许可证)

## 项目亮点

- App 一键启动基础能力：底盘控制、建图、导航、巡逻、相机、目标检测、视频推流和远程控制桥接。
- 语音取物完整链路：唤醒、ASR、声源定位、TTS、LLM 任务理解、物品记忆查询、VLM 目标确认、目标追踪、自动对准、机械臂抓取和返回交付。
- YOLO-VLM-Tracking 多阶段闭环融合：目标检测、VLM 选择、selected-only 追踪、目标丢失重选、HSV 外观匹配、类别约束、速度预测和丢失缓冲。
- 物品记忆支持近似语义匹配，例如用户找“卡皮巴拉玩偶”，记忆库只有“毛绒玩具”时，可以选择相关但不确定的候选并进行说明。
- LLM 任务编排支持复杂复合指令，可完成取物、相对放置、人物交付等类似家庭服务任务，并将自然语言拆解为导航、视觉确认、追踪、抓取、放置和交付等可执行步骤。
- App 支持建图、导航、巡逻、远程摄像头、电量查看、TTS 风格切换和远程语音控制。

## 能力矩阵

| 能力 | 说明 | 主要模块 |
| --- | --- | --- |
| App 基础栈 | 启动底盘、相机、YOLO、导航、建图、巡逻、视频流和 App 桥接 | `app_bridge`、`navigation`、`vision_to_3d` |
| 语音取物 | 理解语音指令，查询物品记忆，抓取物品并返回说话人 | `voice_interaction`、`task_understanding`、`object_memory`、`task_orchestrator` |
| 相对放置 | 将抓取物放到参照物左、右、前、后方 | `vlm_target_selector`、`object_tracker`、`arm_control` |
| 人物交付 | 搜索指定人物，VLM 判断是否符合描述，记录 3D 坐标并导航到安全距离 | `vision_to_3d`、`task_orchestrator`、`navigation` |
| 巡逻记忆 | 在巡逻点扫描可见物体，建立语义物品记忆 | `object_memory`、`vlm_target_selector`、`navigation` |

## 系统架构

系统内部结合 RDK X5 上的地平线 TROS 机器人开发框架和 ROS 中间件完成机器人侧部署。各功能模块通过 Topic、Service 和 Action 解耦。整体可以分为五层：

1. 设备与运行时层：RDK X5、地平线 TROS 机器人开发框架、麦克风阵列、RGB-D 相机、雷达、STM32 底盘控制器、麦克纳姆轮底盘、机械臂、TOF、显示屏和电源模块。
2. 感知理解层：相机采集、YOLO 检测、深度投影、VLM 目标选择、目标追踪和可视化叠加。
3. 语音交互层：语音唤醒、ASR、DOA 声源定位、TTS、人格化播报和音频设备管理。
4. 任务智能层：LLM 任务理解、物品记忆查询、VLM 目标确认、任务规划和上下文记忆。
5. 执行控制层：建图导航、视觉伺服对准、抓取、放置、释放和 App 控制服务。

运行时系统采用事件驱动设计。语音、App 和感知模块发布状态或请求消息，任务总控根据当前状态订阅任务结果并调用下游服务。相机、YOLO、VLM 等计算量较大的模块只在任务需要时激活，从而降低边缘端负载。

```text
用户/App 指令
  -> 语音或 App 桥接
  -> LLM 任务理解
  -> 物品记忆 / VLM 目标确认
  -> 导航与感知
  -> 追踪与视觉伺服对准
  -> 机械臂控制 / 交付 / 放置
  -> TTS 与 App 状态反馈
```

## 目录结构

```text
xiaogua-home-service-robot/
├── README.md                         # 英文项目说明
├── README_cn.md                      # 中文项目说明
├── docs/                             # 技术文档、架构说明和部署说明
│   ├── technical_report.md           # 项目技术报告
│   ├── architecture.md               # 系统架构与模块关系
│   └── quick_start.md                # 快速启动与部署步骤
├── config/                           # 可公开的示例配置
│   ├── navigation/
│   │   └── patrol_points.json        # 巡逻点示例配置
│   └── voice/
│       └── voice_persona.json        # TTS 播报风格配置示例
├── scripts/                          # 常用启动和环境脚本
│   ├── start_app.sh                  # 启动 App 基础栈
│   ├── start_voice_fetch.sh          # 启动语音取物链路
│   └── setup_env.sh                  # 初始化环境变量和工作空间
├── src/                              # 核心功能源码
│   ├── app_bridge/                   # App 通信、状态同步、视频推流和服务桥接
│   ├── voice_interaction/            # 唤醒、ASR、TTS、DOA 和播报风格控制
│   ├── vlm_target_selector/          # LLM 任务解析、物品记忆、VLM 选框和丢失重选
│   ├── object_tracker/               # selected-only 目标追踪和丢失缓冲
│   ├── vision_to_3d/                 # RGB-D 目标三维坐标投影
│   ├── arm_control/                  # 机械臂抓取、释放、收回和视觉伺服对准
│   └── task_orchestrator/            # 取物、放置、找人交付等高层任务状态机
└── tools/
    └── debug/
        └── doa_visualizer/           # 声源方向可视化调试工具
```

## 核心模块说明

| 模块 | 说明 |
| --- | --- |
| `app_bridge` | App 通信桥、命令转发、MJPEG 推流、建图和导航服务封装 |
| `voice_interaction` | 唤醒、ASR、TTS、DOA、语音桥接和播报风格控制 |
| `task_understanding` | 基于 LLM 的意图识别和结构化任务生成 |
| `object_memory` | 物品记忆查询、近似语义匹配和巡逻记忆访问 |
| `vlm_target_selector` | 基于 VLM 的编号候选框目标选择 |
| `object_tracker` | selected-only 目标追踪和丢失处理 |
| `vision_to_3d` | 基于 RGB-D 的目标和人物三维坐标投影 |
| `navigation` | 建图、定位、全局规划、局部规划和巡逻点管理 |
| `arm_control` | 机械臂抓取、释放、收回和待机姿态服务 |
| `task_orchestrator` | 取物、放置、人物交付等高层任务状态机 |

## 重要代码索引

| 文件 | 功能 |
| --- | --- |
| `scripts/start_app.sh` | 启动 App 基础栈，包括底盘、导航、相机、检测、推流和 App 桥接服务。 |
| `scripts/start_voice_fetch.sh` | 在基础栈之上启动语音取物补充链路。 |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/bridge_node.py` | App 主桥接节点，负责 App 指令、机器人状态和服务转发。 |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/mjpeg_server.py` | MJPEG 视频推流服务，用于 App 和浏览器预览。 |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/mapping_service_node.py` | 将建图、定位、导航和巡逻控制封装成 App 侧服务。 |
| `src/app_bridge/robopilot_app_bridge/src/robopilot_app_bridge/robot_cloud_bridge.py` | 云端/MQTT 桥接节点，开源版本应移除真实凭据。 |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/node.py` | XVF3800 麦克风阵列节点，负责音频设备状态、DOA 和语音硬件控制。 |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/wake_word_node.py` | 语音唤醒入口。 |
| `src/voice_interaction/respeaker_xvf3800_ros2/src/respeaker_xvf3800_ros2/asr_client.py` | ASR 语音识别请求封装。 |
| `src/voice_interaction/asr_bridge/asr_ros1_bridge/tts_host_node.py` | TTS 播放主机和语音反馈节点。 |
| `src/voice_interaction/asr_bridge/asr_ros1_bridge/persona_control_node.py` | App 和语音反馈中的播报风格/人格切换。 |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/task_understanding_node.py` | LLM 任务解析节点，将自然语言转换成结构化 intent、任务字段和可执行 plan。 |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/memory_query_node.py` | 物品记忆查询节点，支持不完美物品名的近似语义匹配。 |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/selector_node.py` | VLM 目标选择节点，从编号 YOLO 候选框中选出目标。 |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/target_confirm_node.py` | VLM 目标确认节点，用于执行前确认模糊视觉目标。 |
| `src/vlm_target_selector/vlm_target_selector/vlm_target_selector/lost_reselector_node.py` | 目标丢失重选节点，在追踪丢失后重新调用 VLM 选择。 |
| `src/vlm_target_selector/yolo_detector/yolo_detector/detector_node.py` | YOLO 相机目标检测节点。 |
| `src/vlm_target_selector/yolo_detector/yolo_detector/yolo_engine.py` | YOLO 检测模型推理封装。 |
| `src/object_tracker/object_tracker/object_tracker/tracker_node.py` | selected-only 目标追踪节点，包含关联、外观匹配、预测和丢失缓冲。 |
| `src/vision_to_3d/vision_to_3d_local/vision_to_3d_local/vision_to_3d_local_node.py` | 将 2D 检测框和深度图投影为物体/人物 3D 坐标。 |
| `src/vision_to_3d/vision_tf_bridge/vision_ros1_tf_bridge/selected_detection_bridge_node.py` | 只把 VLM 选中的目标检测转发给执行侧，用于对准和抓取。 |
| `src/arm_control/auto_aim.py` | 抓取或放置前的分段式 PID 视觉伺服对准。 |
| `src/arm_control/arm_node.py` | 机械臂服务节点，提供抓取、释放、收回和待机动作。 |
| `src/task_orchestrator/voice_fetch_orchestrator.py` | 取物、相对放置、返回说话人、人物交付的高层任务状态机。 |
| `src/task_orchestrator/voice_person_follow_flow.py` | “过来一下”等指令使用的说话人/人物定位与导航流程。 |

## 主要工作流程

项目提供两类主要启动入口。实际部署时通常先启动 App 基础栈，再按任务需要启动语音取物补充链路。

### App 基础能力

App 启动后开启机器人基础能力：

```text
底盘 -> 导航 -> 建图/巡逻 -> 相机 -> YOLO -> 视频流 -> App 桥接
```

示例命令：

```bash
source scripts/setup_env.sh
bash scripts/start_app.sh
```

### 语音取物

语音取物链路在基础能力之上补充完整的语音任务执行流程：

```text
唤醒/ASR -> 任务理解 -> 记忆查询 -> 导航 -> VLM 确认
-> 目标追踪 -> 自动对准 -> 抓取 -> 返回/交付
```

示例命令：

```bash
source scripts/setup_env.sh
bash scripts/start_voice_fetch.sh
```

### 指令示例

任务理解模块会保留关键语义修饰，而不是只提取主物品类别。

| 用户指令 | 预期执行路线 |
| --- | --- |
| “把卧室的水拿给我” | 记录说话人位置，去卧室找水，抓取，返回说话人 |
| “把小兔子玩偶放到客厅维生素片右边” | 找并抓取玩偶，导航到客厅，确认维生素片，对准右方并释放 |
| “过来一下” | 记录声源方向和人物坐标，导航到说话人附近 |
| “你知道卡皮巴拉玩偶在哪吗” | 查询物品记忆；没有精确匹配时允许近似匹配毛绒玩具 |
| “把卧室的水拿给正在拖地的人” | 找并抓取水，搜索指定人物，VLM 判断后导航到安全距离 |

## 关键算法

### 建图与导航

- Cartographer 建图
- Occupancy Grid 栅格地图
- 基于地图的定位链路
- Global Costmap 和 Local Costmap
- Dijkstra 全局路径规划
- DWA 局部路径规划
- 导航状态机和巡逻点管理

### YOLO-VLM-Tracking

感知链路将目标检测、视觉语言推理和目标追踪结合起来：

- YOLO 目标检测
- VLM 从编号候选框中选择目标
- selected-only 目标桥接
- ByteTrack-style 两阶段关联
- HSV 外观特征匹配
- 类别约束
- 速度预测
- 丢失缓冲
- 目标丢失后的 VLM 重选

### 语音与任务理解

- 语音唤醒
- ASR 语音识别
- DOA 声源定位
- TTS 语音反馈
- LLM 结构化任务解析
- 物品记忆查询
- 近似语义匹配

### LLM 任务编排 Plan

任务智能模块不只是判断用户意图，还会把自然语言指令转换成结构化任务 JSON。这个 JSON 同时包含高层意图字段和可执行的逐步 `plan`。

典型字段包括：

- `intent`：任务类型，例如 `fetch_to_speaker`、`transfer_object`、`deliver_to_person`、`come_to_speaker`、`navigate_to`、`task_chain`、`chat`
- `target_name`：完整目标描述，会保留“遥控器旁边的手机”“最右边那瓶水”等空间修饰
- `semantic_hint`：目标类别和检测线索，例如 `bottle`、`cup`、`book`、`teddy bear`
- `source_location` 和 `destination_location`：物品来源地点和目标地点
- `delivery_target`：说话人、固定地点或人物描述
- `placement_reference` 和 `placement_side`：相对放置任务中的参照物和方位
- `tasks`：面向当前执行器的简化兼容步骤
- `plan`：更完整的语义动作序列，用于任务编排

`plan` 会把任务拆成可执行语义步骤，例如：

```json
[
  {"action": "find_object", "location": "卧室", "target": "水", "search_locations": ["卧室"]},
  {"action": "grasp_object", "location": "卧室", "target": "水"},
  {"action": "find_person", "target": "正在拖地的人"},
  {"action": "navigate_to", "location": "person", "target": "正在拖地的人"}
]
```

任务总控会把这些语义步骤映射到机器人能力：

- `find_object`：查询物品记忆，导航到记忆点；记忆缺失时使用候选地点搜索
- `grasp_object`：调用 VLM 确认目标，选择目标框，启动追踪、视觉伺服对准和机械臂抓取
- `navigate_to`：导航到固定地点或已经记录的人物坐标
- `place_relative`：寻找放置参照物，并把当前夹持物对准到参照物指定方位
- `find_person`：搜索指定人物，使用 VLM 判断是否符合描述，记录三维坐标并导航到安全距离
- `return_to_speaker`：回到声源定位阶段记录的说话人位置

通过这种方式，系统可以用同一套语言接口处理导航、取物、搬运、相对放置、记忆查询和人物交付，而不需要为每一句话硬编码规则。

## 配置说明

部署相关值建议通过环境变量或 launch 参数配置，不建议硬编码到代码中。

| 变量 | 作用 | 示例 |
| --- | --- | --- |
| `ROBOT_IP` | 机器人对外访问地址 | `robot-ip.example` |
| `YOLO_MODEL_PATH` | 本地目标检测模型路径 | `/opt/xiaogua/models/yolo_model.bin` |
| `YOLO_CLASS_NAMES` | 目标检测类别列表 | 常见家庭物品类别等 |
| `DASHSCOPE_API_KEY` | 使用云端模型时的 API Key | `your_api_key_here` |
| `VLM_URL` | 使用本地 VLM 时的服务地址 | `http://127.0.0.1:8000/analyze` |
| `LLM_URL` | 使用本地 LLM 时的服务地址 | `http://127.0.0.1:8000/analyze` |
| `MQTT_HOST` | 可选云端消息代理地址 | `your_mqtt_host` |

推荐先加载环境占位配置：

```bash
source scripts/setup_env.sh
```

随后根据自己的机器人平台修改 `scripts/setup_env.sh`，或者在启动前自行导出环境变量。


## 模型配置

项目可以切换云端模型或本地模型。参考配置如下：

- 任务理解、记忆推理和播报生成：`qwen3.6-flash`
- 视觉语言目标选择：`qwen3-vl-plus`
- TTS：`qwen3-tts-instruct-flash-realtime`
- ASR：`qwen3-asr-flash`

模型名称均应做成配置项，开源部署时可以按实际环境替换。

## 硬件参考

- RDK X5 边缘计算平台
- RDK X5 上的地平线 TROS 机器人开发框架
- XVF3800 麦克风阵列
- Astra Plus Pro RGB-D 相机
- STM32 底盘控制器
- 4ROS 雷达
- TOF 传感器
- 六自由度机械臂
- 麦克纳姆轮底盘
- LED 显示屏
- DCDC 降压稳压模块

## 文档说明

更多文档可以放在 `docs/` 目录：

- `docs/technical_report.md`：项目技术报告和系统设计细节
- `docs/architecture.md`：模块级架构说明
- `docs/quick_start.md`：启动说明和本地配置检查表
