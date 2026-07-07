# 基于 RDK X5 的端云协同多模态具身智能家庭服务机器人“小瓜”技术报告

## 摘要

“小瓜”是一套基于 RDK X5 主控平台构建的端云协同多模态具身智能家庭服务机器人。系统面向真实家庭或实验室环境中的建图、导航、巡逻、远程看护、语音交互、语音取物、搬运和相对放置任务，构建了一套基于 ROS 模块化架构的完整具身智能系统。系统本地侧负责传感器接入、导航控制、BPU 视觉推理、目标跟踪、底盘对准和机械臂执行；云端侧负责 LLM 任务理解、VLM 语义目标确认、TTS 语音合成和远程通信入口。

技术路线可以概括为：

```text
语音自然语言输入
  -> LLM 结构化任务理解
  -> 物品记忆与现场搜索结合
  -> YOLO-VLM-Tracking 融合目标锁定
  -> 分段式 PID 视觉伺服对准
  -> 机械臂抓取/释放
  -> TTS 多风格反馈
```

该方案的核心特点是将大模型用于高层理解与语义选择，将传统 ROS 控制链路用于确定性执行，形成“云端语义智能 + 本地实时控制”的分层闭环。系统既支持 App 便捷建图、导航、巡逻和远程查看，也支持普通取物、跨地点搬运、相对放置、记忆失败后的现场搜索、多风格语音播报和远程语音控制。

## 1. 项目概述

本项目面向家庭服务机器人在真实室内环境中的“建图、导航、巡逻、远程看护、语音取物、抓取、搬运、相对放置”任务。系统在 App 一键启动已经运行的前提下，复用底盘、导航、建图、相机、YOLO、视频推流和 ROS 通信桥等基础能力，并补充语音交互、任务理解、视觉确认、目标跟踪、分段式 PID 视觉伺服对准、机械臂抓取和任务编排节点。

系统的核心目标不是单点完成“识别一个物体”，而是将语音理解、场景记忆、视觉确认、底盘导航、机械臂抓取、相对放置和状态播报串成一个可运行的闭环流程。用户可以通过自然语言表达任务，例如：

- “帮我拿一下矿泉水。”
- “帮我把快递放到矿泉水左边。”
- “帮我把卧室最靠近卡皮巴拉玩偶的农夫山泉放到客厅靠近比比赞零食箱的维生素片左边。”
- “回去吧。”
- “切换成话痨风格。”

系统通过 LLM 进行任务理解和流程编排，通过 VLM 辅助视觉目标选择，通过 YOLO 提供实时候选框，通过 ByteTrack-style 关联、selected-only 约束和 HSV 外观匹配组成的多策略追踪算法维持目标锁定，通过分段式 PID 视觉伺服算法实现底盘精对准，通过机械臂服务完成抓取、释放和收回动作。

系统中的大模型均采用云端百炼模型，不依赖本地 VLM：

| 模块 | 用途 | 模型 |
| --- | --- | --- |
| 任务理解 LLM | 意图识别、任务拆解、plan 编排 | `qwen3.6-flash` |
| 记忆查询 LLM | 物品名称语义匹配、候选排序 | `qwen3.6-flash` |
| 播报生成 LLM | 根据 persona 生成任务播报词 | `qwen3.6-flash` |
| 视觉理解 VLM | 在 YOLO 候选框中选择目标、确认目标语义 | `qwen3-vl-plus` |
| 语音合成 TTS | 将文本播报合成为语音 | `qwen3-tts-instruct-flash-realtime` |
| 语音识别 ASR | 将用户语音转成文本 | `qwen3-asr-flash` |

## 2. 系统运行场景

系统采用“App 主流程 + 语音取物补充流程”的共存架构。

## 2.1 硬件平台组成

本系统运行在地瓜 RDK X5 主控平台上，整机由感知、计算、运动控制、执行机构和交互显示等硬件模块组成。

| 硬件模块 | 作用 |
| --- | --- |
| RDK X5 主控 | 机器人主计算平台，运行 ROS、App 桥接、YOLO BPU 推理、语音取物流程和各类任务节点 |
| reSpeaker Flex XVF3800 阵列麦克风 | 负责语音唤醒、用户语音采集、声源方向 DOA 和 VAD 检测 |
| Astra Plus Pro 深度相机 | 提供 RGB 图像和深度图，用于 YOLO 检测、VLM 目标确认、目标距离估计和三维定位 |
| STM32 底层控制板 | 负责底盘电机、机械臂、IMU 等底层实时控制和状态采集，执行上层速度与动作指令，并与主控通过串口通信 |
| 4ROS 激光雷达 | 提供环境扫描数据，用于 Cartographer 建图、定位、导航和避障相关能力 |
| TOF 激光测距模块 | 提供近距离测距信息，可用于靠近目标、辅助避障或抓取前距离判断 |
| 六自由度机械臂 | 执行取物、抓取、释放、收回等动作，是语音取物任务的末端执行机构 |
| 麦克纳姆轮底盘 | 支持全向移动，使机器人可以前后、横移和旋转，便于视觉对准和相对放置 |
| LED 屏幕 | 用于机器人状态显示、人机交互提示或任务反馈展示 |
| DCDC 降压稳压模块 | 将电池或上级电源转换为各模块所需稳定电压，为主控、传感器、底层控制板和执行机构提供可靠供电 |

硬件与软件模块的对应关系如下：

```text
XVF3800 阵列麦克风
  -> 唤醒 / ASR / DOA / VAD

Astra Plus Pro 深度相机
  -> RGB 图像 / 深度图 / YOLO / VLM / 三维定位

RDK X5
  -> ROS 模块化系统 / BPU YOLO 推理 / App 桥接 / 任务编排

STM32 底层控制板
  -> 底盘电机控制 / 机械臂底层控制 / IMU 姿态数据 / 串口通信 / 底层状态反馈

麦克纳姆轮底盘
  -> /cmd_vel 执行 / 分段式 PID 视觉伺服对准 / 导航运动

4ROS 激光雷达
  -> Cartographer 建图 / 定位 / 导航 / 巡逻

TOF 激光测距模块
  -> 近距离测距 / 辅助靠近 / 安全判断

六自由度机械臂
  -> /execute_grasp / /execute_release / /execute_retract

LED 屏幕
  -> 状态显示 / 交互反馈

DCDC 降压稳压模块
  -> 电源转换 / 电压稳定 / 为主控、传感器和执行机构供电
```

这套硬件组合使机器人同时具备语音交互、视觉识别、深度感知、激光导航、全向移动、机械臂抓取、底层姿态感知和稳定供电能力，为“听懂任务、找到物品、靠近目标、抓取物品、搬运放置”提供完整闭环。

## 2.2 服务器构建与通信架构

系统除了机器人本体上的 ROS 节点外，还包含公网服务器和多条通信链路，用于 App 远程访问、视频查看、消息桥接和大模型调用。整体通信架构可以分为四层：

```text
App / 浏览器 / 控制端
  -> 公网服务器 frps / MQTT / WebSocket
  -> RDK X5 frpc / 本地 App 服务
  -> ROS 模块化机器人系统
  -> 底盘、相机、机械臂、语音和传感器硬件
```

### 2.2.1 公网服务器侧

公网服务器用于承载远程访问入口，主要包含：

| 服务 | 端口 | 作用 |
| --- | --- | --- |
| frps | `1935` | frp 服务端，接收 RDK X5 上 frpc 的反向连接 |
| MJPEG 公网转发端口 | `8888` | 将机器人本地 `8081` 视频流映射到公网 |
| MQTT TCP | `1883` | App/云端消息通信 |
| MQTT WebSocket | `9001` | 浏览器或 App 通过 WebSocket 接入 MQTT |

公网视频访问地址：

```text
http://public-server.example:8888/stream
```

服务器侧的核心作用不是运行机器人控制算法，而是提供稳定的公网入口。机器人仍然在 RDK X5 本地执行导航、视觉、抓取和任务编排，公网服务器只承担通信转发和远程访问入口角色。

### 2.2.2 RDK X5 机器人侧 frpc

RDK X5 上运行 frpc 客户端，将本地视频服务转发到公网服务器：

```toml
serverAddr = "public-server.example"
serverPort = 1935

[[proxies]]
name = "robot_mjpeg"
type = "tcp"
localIP = "127.0.0.1"
localPort = 8081
remotePort = 8888
```

本地视频服务地址：

```text
http://127.0.0.1:8081/stream
http://robot-ip.example:8081/stream
```

公网视频服务地址：

```text
http://public-server.example:8888/stream
```

frpc 可通过 systemd 托管，服务名：

```text
xiaogua-frpc.service
```

这样机器人开机或 App 一键启动后可以自动恢复公网视频转发，不需要每次手动运行 frpc。

### 2.2.3 本地 App 与机器人通信

机器人本地保留多条通信通道，分别服务于 App、语音取物和底层执行：

| 通道 | 地址/端口 | 作用 |
| --- | --- | --- |
| App ROS rosbridge | `ws://0.0.0.0:9090` | App 与 ROS 系统通信 |
| 语音取物 rosbridge | `ws://127.0.0.1:9091` | ASR/DOA/TTS 与执行侧桥接 |
| App MJPEG 视频 | `http://0.0.0.0:8081/stream` | 本地视频预览与公网 frp 转发源 |
| App 速度控制 WebSocket | `ws://0.0.0.0:19091` | App 手动速度控制 |
| ROS topic/service | 本机 ROS 通信 | 导航、视觉、机械臂、任务状态 |

端口隔离的目的：

- `9090` 保留给 App，避免语音取物流程抢占 App 通信。
- `9091` 专门给语音取物桥接，ASR、DOA、任务消息互不干扰。
- `8081` 只作为本地 MJPEG 视频源，公网访问通过 frp 映射到 `8888`。
- `19091` 专门用于 App 手动速度控制，执行取物任务时应避免同时手动遥控。

### 2.2.4 大模型云端通信

语言理解、视觉理解、播报生成、TTS 和 ASR 均通过百炼云端接口完成。

| 能力 | 模型 | 通信方式 |
| --- | --- | --- |
| 任务理解 | `qwen3.6-flash` | HTTPS API |
| 记忆语义匹配 | `qwen3.6-flash` | HTTPS API |
| 播报生成 | `qwen3.6-flash` | HTTPS API |
| VLM 目标确认 | `qwen3-vl-plus` | HTTPS API，上传图像和候选框信息 |
| TTS 语音合成 | `qwen3-tts-instruct-flash-realtime` | WebSocket 流式合成 |
| ASR 语音识别 | `qwen3-asr-flash` | HTTPS/API 调用 |

由于 TTS 使用 WebSocket 流式合成，对网络抖动更敏感。系统已将 Qwen TTS WebSocket 连接等待时间从 5 秒提升到 10 秒，降低偶发网络慢导致播报失败的概率。

### 2.2.5 通信链路总结

完整通信链路可以概括为：

```text
远程视频:
  App/浏览器
    -> http://public-server.example:8888/stream
    -> frps:1935
    -> RDK X5 frpc
    -> 本地 MJPEG 8081
    -> 相机图像

App 控制:
  App
    -> 9090 rosbridge / 19091 WebSocket
    -> ROS App 桥接节点
    -> 机器人状态、模式、速度控制

语音取物:
  XVF3800 / ASR / DOA
    -> 9091 语音 rosbridge
    -> voice_fetch_orchestrator
    -> LLM/VLM/YOLO/tracker/分段式 PID 视觉伺服/arm_node

大模型:
  RDK X5 任务节点
    -> 百炼 HTTPS / WebSocket
    -> qwen3.6-flash / qwen3-vl-plus / qwen3-tts / qwen3-asr
```

该通信设计的特点是：本地执行链路保持闭环，公网服务器只做远程入口和消息转发；机器人核心控制不依赖公网服务器运行，只有大模型调用和远程访问需要外部网络。

App 一键启动命令：

```bash
cd /opt/xiaogua/ros2_ws
bash src/robopilot_app_bridge/scripts/start_robopilot_app.sh
```

App 一键启动已经提供：

- ROS rosbridge，端口 `9090`
- App MJPEG 视频，端口 `8081`
- App 速度控制 WebSocket，端口 `19091`
- Astra 相机
- YOLO detector
- mapping_service_node
- cmd_vel_relay
- 底盘 base.launch
- 模式管理 mode_manager.launch
- robopilot_ros1_app_bridge.py
- 巡逻扫描和物品记忆能力

App 侧面向普通用户提供便捷操作入口，主要能力包括：

| App 能力 | 说明 |
| --- | --- |
| 便捷建图 | 通过 App 切换建图模式，基于 Cartographer 完成环境地图构建，并保存地图文件 |
| 导航控制 | 通过 App 切换导航模式，基于已有地图进行定位和目标点导航；全局路径规划采用 Dijkstra，局部避障与速度规划采用 DWA |
| 巡逻扫描 | 通过 App 启动巡逻任务，机器人按点位移动并扫描环境中的物品，形成物品记忆 |
| 远程摄像头查看 | 通过 MJPEG 视频流查看机器人当前画面，局域网和公网均可访问 |
| 电量查看 | App 读取机器人状态信息并展示电量，辅助判断任务执行和充电需求 |
| TTS 播报风格切换 | App 通过 `/voice_persona/set` 切换默认、管家、活泼、话痨、自定义等播报风格 |
| 远程语音控制 | App/远程端可发送语音或文本任务，让机器人执行取物、搬运、返回等任务 |
| 手动速度控制 | App 可通过速度控制 WebSocket 遥控底盘运动，用于调试或人工接管 |

App 不是单纯的遥控器，而是机器人家庭服务入口。它负责把建图、导航、巡逻、视频、电量、语音风格和远程控制能力整合成用户可操作界面；机器人本体则负责本地实时控制和任务执行。

语音取物补充启动命令：

```bash
cd /opt/xiaogua/ros2_ws
bash src/robopilot_app_bridge/scripts/start_voice_fetch_autoaim.sh
```

补充流程不会重复启动底盘、导航、相机、YOLO、视频服务和 App rosbridge，而是只补充：

- 语音专用 rosbridge，端口 `9091`
- ASR/DOA/TTS 桥接
- LLM 任务理解节点
- 物品记忆查询节点
- VLM 目标确认节点
- VLM 目标选择节点
- object_tracker
- selected_detection_bridge
- 分段式 PID 视觉伺服对准节点
- arm_node.py
- voice_fetch_orchestrator

## 3. 总体架构

系统采用 ROS 模块化架构。App、相机、YOLO、VLM、LLM、语音桥接、底盘导航、分段式 PID 视觉伺服对准、机械臂服务和取物总控被拆分为独立节点，通过标准话题、服务和桥接接口组合成完整任务链路。报告中不强调底层 ROS 版本差异，重点关注模块边界、数据接口和任务编排。

运行环境上，系统基于 ROS Humble 工作空间，并在地瓜 RDK X5 环境中兼容加载 TROS：

```bash
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
source /opt/xiaogua/ros2_ws/install/setup.bash
```

TROS 主要用于适配 RDK X5 的机器人开发环境、BPU 推理链路和 `ai_msgs` 等消息/组件能力。YOLO `.bin` 模型运行在 RDK X5 BPU 上，App、视觉和三维定位相关节点可复用 TROS 环境提供的接口；任务理解、VLM、TTS 等大模型能力仍走百炼云端模型。

### 3.1 关键模块接口

| 模块 | 输入 | 输出 | 作用 |
| --- | --- | --- | --- |
| reSpeaker/ASR | 用户语音 | `/asr_command` | 将语音转换为文本任务 |
| DOA/VAD | 麦克风阵列信号 | `/xvf3800/doa_deg`、`/xvf3800/vad` | 记录说话人方向 |
| task_understanding_node | 文本指令、最近任务摘要 | 结构化 intent/plan | LLM 任务理解与编排 |
| object_memory_query_node | 目标名称 | 记忆候选点位 | 查询物品空间记忆 |
| YOLO detector | RGB 图像 | detections | 生成实时候选框 |
| target_confirm / vlm_target_selector | 图像、候选框、目标描述 | selected_detection | VLM 语义选框 |
| object_tracker | YOLO detections、selected_detection | `/object_tracker/selected_detection` | 持续跟踪目标 |
| selected_detection_bridge | 跟踪目标 | `/tracked_yolov8/detections` | 转发给执行侧 |
| 分段式 PID 视觉伺服对准 | 跟踪目标框、TOF、激光雷达 | `/cmd_vel`、`/red_align_success` | 四阶段靠近、转正、贴近和像素级横向对准 |
| arm_node.py | 抓取/释放/收回服务请求 | 服务执行结果 | 控制机械臂动作 |
| voice_fetch_orchestrator | ASR 指令、各模块结果 | 任务状态、服务调用 | 总控调度 |
| TTS/persona | 状态文本、风格配置 | 语音播报 | 多风格反馈 |

总体数据流如下：

```text
用户语音 / App 手动指令
  -> reSpeaker 唤醒 / ASR / DOA
  -> ASR/DOA 桥接到取物总控
  -> voice_fetch_orchestrator 接收任务
  -> 记录说话人方向与地图位置
  -> task_understanding_node 调用 qwen3.6-flash 理解任务
  -> object_memory_query_node 查询 object_memory.json
  -> move_base 导航到记忆点或指定地点
  -> YOLO 输出候选目标
  -> target_confirm_node / vlm_target_selector 调用 qwen3-vl-plus 确认目标
  -> object_tracker 跟踪选中目标
  -> selected_detection_bridge 转发给执行侧
  -> 分段式 PID 视觉伺服控制底盘完成精对准
  -> arm_node.py 调用机械臂抓取/释放/收回
  -> 返回说话人位置或执行相对放置
  -> TTS 播报任务状态
```

## 4. 模块化设计

### 4.1 App 基础模块

App 基础模块负责机器人基础运行能力，包括相机、视频、YOLO、底盘桥接、建图、导航模式管理和 App 控制接口。

建图、定位与导航算法：

| 层级 | 算法/模块 | 作用 | 在系统中的使用方式 |
| --- | --- | --- | --- |
| 建图 | Cartographer | 基于激光雷达扫描和机器人运动数据构建二维栅格地图 | App 进入建图模式后完成室内地图构建，并保存为后续导航地图 |
| 地图表示 | Occupancy Grid 栅格地图 | 将环境表示为可通行、障碍、未知区域 | 供全局规划、局部代价地图和巡逻点位使用 |
| 定位 | 基于已保存地图的激光定位链路 | 估计机器人在地图中的实时位姿 | 导航、巡逻和语音取物均复用该定位结果 |
| 全局代价地图 | Global Costmap | 融合静态地图、障碍物膨胀和机器人半径，形成全局可规划区域 | 为全局路径规划提供代价空间 |
| 全局规划 | Dijkstra | 在全局代价地图上搜索从当前位置到目标点的可行路径 | 用于导航到巡逻点、物品记忆点、用户指定地点和说话人位置 |
| 局部代价地图 | Local Costmap | 根据机器人附近的实时障碍物更新局部可通行区域 | 为局部避障和短时速度规划提供约束 |
| 局部规划 | DWA | 在速度空间中采样线速度、角速度，结合障碍物距离、路径跟随和目标方向选择最优速度 | 输出实时 `/cmd_vel`，完成避障、跟随全局路径和到点收敛 |
| 执行接口 | move_base / 导航状态机 | 接收目标点并调度全局规划、局部规划和恢复行为 | 语音任务只调用导航能力，不重复启动底盘、雷达或导航节点 |

语音取物流程不重新启动建图或导航节点，而是复用 App 一键启动后的定位、地图、全局规划和局部规划能力。这样可以避免重复启动雷达、map_server、move_base 或底盘节点导致的资源竞争，也保证 App 导航、巡逻和语音取物使用同一套地图坐标系。

设计原则：

- 语音取物流程不重复启动 App 已有节点。
- App 继续保留手动控制、建图、导航、巡逻能力。
- 语音取物流程通过独立脚本补充，不破坏 App 的生命周期。
- App 作为普通用户入口，整合便捷建图、导航、巡逻、远程摄像头查看、电量显示、TTS 播报风格切换和远程语音/文本控制。
- 本地 MJPEG 视频服务既供局域网 App 查看，也通过 frpc 映射到公网，便于远程观察机器人视角。

关键端口：

```text
9090  App ROS rosbridge
9091  语音取物专用 rosbridge
8081  App MJPEG 视频
19091 App 速度控制 WebSocket
```

该设计避免了多个 rosbridge、多个相机驱动、多个 move_base 或多个雷达节点重复启动导致的资源竞争。

### 4.2 语音输入模块

语音输入模块由 reSpeaker XVF3800、唤醒词节点、ASR 节点、DOA 节点和 ROS 桥接组成。

功能：

- 唤醒机器人。
- 录制用户语音。
- 调用 ASR 得到文本指令。
- 获取声源方向 DOA。
- 将 ASR 文本转发到执行侧 `/asr_command`。
- 将 DOA/VAD 信息转发到执行侧 `/xvf3800/doa_deg` 和 `/xvf3800/vad`。

工程优化：

- ASR 和 DOA 使用独立语音 rosbridge 端口 `9091`，避免占用 App 的 `9090`。
- 增加 USB autosuspend 禁用，降低 XVF3800、USB hub、串口设备掉线概率。
- 对 XVF3800 做启动健康检查和重枚举，检测 PCM zero-filled 异常。
- TTS 播放状态会影响 ASR 录音，避免机器人自己播放的语音被误识别。

当前语音识别模型：

```text
qwen3-asr-flash
```

### 4.3 TTS 播报模块

TTS 模块负责将系统状态、任务反馈、风格切换确认语合成为语音。

当前语音合成模型：

```text
qwen3-tts-instruct-flash-realtime
```

当前播报词生成模型：

```text
qwen3.6-flash
```

支持能力：

- 固定短句播报。
- LLM 生成播报词。
- 多播报风格切换。
- App 发送 `/voice_persona/set` 后切换 persona。
- 自定义风格可调用 LLM 生成一句符合风格的确认播报。

当前支持的风格包括：

- `default`：默认家用助手
- `calm_butler`：沉稳管家
- `playful_partner`：活泼搭档
- `chatty_funny`：话痨搞笑
- `custom`：App 自定义风格

角色设定为机器人“小瓜”。播报风格配置集中在：

```text
/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json
```

创新点：

- 将“任务事实”和“表达风格”分离。LLM 可以生成不同口吻，但不能改变任务成败、物品名称和位置事实。
- 允许 App 动态切换播报风格，不需要重启整个机器人。
- 固定播报和 LLM 播报混合使用，避免每一句都依赖云端，兼顾娱乐性和稳定性。

### 4.4 LLM 任务理解模块

`task_understanding_node` 负责将自然语言指令转换为结构化任务。

输入：

```text
用户原始语音文本
最近任务上下文
当前机器人状态
```

输出：

```json
{
  "intent": "fetch_object / transfer_object / place_relative / return_home / chat / unknown",
  "target_name": "目标物体",
  "source_location": "起点位置",
  "destination": "目标位置",
  "placement_reference": "相对放置参照物",
  "placement_side": "left/right/front/back",
  "plan": [
    {"action": "navigate_to", "target": "..."},
    {"action": "find_object", "target": "..."},
    {"action": "grasp_object", "target": "..."},
    {"action": "find_object", "target": "..."},
    {"action": "place_relative", "reference": "...", "side": "..."},
    {"action": "release_object"}
  ],
  "tts_text": "播报词"
}
```

LLM 不是直接控制机器人，而是输出受限 JSON。取物总控读取 JSON 后按白名单动作执行，避免 LLM 任意控制底盘或机械臂。

当前任务理解模型：

```text
qwen3.6-flash
```

关键设计：

- 复杂任务必须生成显式 plan。
- 相对放置任务必须包含 `find_object(reference)` 和 `place_relative`。
- 当前明确指令优先，最近上下文只用于“刚才那个”“放回去”等省略表达。
- 若用户说了前置地点，即使记忆库没有目标，也要去该地点进行 YOLO + VLM 搜寻。
- LLM 只负责理解和编排，不直接发布 `/cmd_vel`。
- 对 LLM 首次输出增加一次性 plan 一致性校验，发现字段和 plan 不一致时只允许修复一次，避免反复重试造成延迟或循环。

一次性计划校验的典型检查项包括：

- 如果 `placement_reference` 和 `placement_side` 已经存在，plan 必须包含 `place_relative`。
- 如果需要相对放置，plan 中必须先 `find_object(reference)`，再执行 `place_relative(reference, side)`，最后再 `release_object`。
- 如果用户给出明确目标地点，plan 中必须包含对应的 `navigate_to`。
- 如果需要抓取，`grasp_object` 前必须有对应目标的 `find_object`。
- 如果需要返回说话人位置，plan 中必须包含 `return_to_speaker` 或等价返回步骤。

校验器只检查结构一致性，不写死具体物品名称和任务规则。若修复失败，系统保留首次 LLM 输出并在日志中记录 `plan_validation` 元数据，便于排查。

创新点：

- 将 LLM 定位为“任务编排器”，而不是底层控制器。
- 通过结构化 plan 把自然语言任务拆成导航、识别、抓取、放置、返回等可验证步骤。
- 通过最近 2 条任务摘要支持轻量上下文记忆，避免每次把大量历史对话发送给 LLM。
- 将相对空间语言转为可执行视觉对准参数，例如“放到矿泉水左边”转换为让矿泉水位于画面右侧。
- 通过一次性 plan 校验修复机制，把 Prompt 约束从“纯提示”升级为“提示 + 结构审查”，提高复杂搬运和相对放置任务稳定性。

### 4.5 任务记忆模块

系统有两类记忆：

第一类是物品空间记忆：

```text
/opt/xiaogua/data/patrol_memory/object_memory.json
```

它记录巡逻扫描得到的物品名称、可能名称、YOLO 类别、巡逻点、地图位置等。

第二类是任务摘要记忆：

```text
/opt/xiaogua/data/robopilot_memory/task_memory.json
```

它只记录最近任务的简短摘要，默认只取最近 2 条传给 LLM。

设计原因：

- 物品空间记忆用于“去哪里找物品”。
- 任务摘要记忆用于“理解上下文省略”。
- 不把所有历史都传给 LLM，降低 token、延迟和误导风险。

创新点：

- 将长期空间记忆和短期任务上下文分开管理。
- 记忆库不是硬约束。记忆查不到时仍可根据用户指定地点进行现场搜索。
- 通过“任务摘要 + 最近上下文”支持更自然的人机对话。

### 4.6 物品记忆查询模块

`object_memory_query_node` 根据 LLM 输出的目标名称查询物品记忆库。

当前记忆查询语义匹配模型：

```text
qwen3.6-flash
```

功能：

- 精确名称匹配。
- possible_names 匹配。
- LLM 辅助语义匹配。
- 返回候选物品位置、巡逻点、名称和置信信息。

当记忆命中：

```text
直接导航到记忆点 -> 视觉确认 -> 抓取
```

当记忆未命中但用户给出地点：

```text
导航到用户指定地点 -> 打开现场 YOLO/VLM 搜索 -> 找到后继续任务
```

当记忆未命中且没有地点：

```text
提示没有找到物品，任务失败或请求补充信息
```

### 4.7 YOLO 视觉检测模块

YOLO detector 提供实时候选框。系统使用自定义 BPU `.bin` 模型，可检测常见取物对象。

当前自定义类别示例：

```text
person
cell phone
mouse
remote
book
bottle
cup
bowl
apple
banana
teddy bear
bag_wrapper
box
```

关键配置：

- 模型路径：`/opt/xiaogua/models/yolo_model.bin`
- 输入图像：`/camera/color/image_raw`
- 置信度：`0.07`
- 预处理：`letterbox`
- 推理图像：640x480 色彩图

工程优化：

- 避免 camera_mux 双重发布大图导致高 CPU。
- YOLO 直接订阅真实相机 topic。
- VLM 和目标确认内部通过 task gate 控制，避免空闲时持续推理。
- 低置信度候选交给 VLM 二次判断，提高复杂场景下的召回。

创新点：

- YOLO 不负责最终语义判断，只负责提供候选区域。
- VLM 结合用户目标描述在候选框中选择目标，解决“多个瓶子”“靠近某物的瓶子”“纸盒左侧的矿泉水”等细粒度指代问题。
- 允许开全类别识别，在记忆失败时进行现场搜索。

### 4.8 VLM 目标确认模块

VLM 目标确认由 `target_confirm_node` 和 `vlm_target_selector` 完成。

当前 VLM 模型：

```text
qwen3-vl-plus
```

该模块不使用本地 VLM，统一调用百炼视觉模型。YOLO 负责提供候选框，VLM 只在候选框范围内做语义选择和目标确认。

输入：

- 当前相机图像
- YOLO 候选框
- 用户目标名称
- 任务上下文

输出：

- 目标是否存在
- 选中的候选编号
- 目标语义描述
- 选中目标的检测框

典型流程：

```text
YOLO 检测到多个候选
  -> 给每个候选画编号
  -> VLM 根据任务描述选择目标编号
  -> selector_node 输出 selected_detection
  -> object_tracker 接管目标
```

优化点：

- Prompt 要求 VLM 只输出最终编号，减少解释文本干扰。
- 解析逻辑优先读取“答案：1”“最终答案：1”“选择 1”等结论数字。
- 如果找不到结论数字，才兜底取最后一个有效数字。

创新点：

- 将“目标检测”和“目标理解”拆开。YOLO 负责快，VLM 负责准。
- VLM 不直接操作机器人，只输出候选编号，便于审计和回放。
- 支持相对语义，如“最靠近卡皮巴拉玩偶的矿泉水”“靠近比比赞零食箱的维生素片”。

### 4.9 多策略融合目标跟踪模块

object_tracker 在 VLM 选中目标后进行连续跟踪。该模块不是单纯把所有 YOLO 框逐帧画出来，而是围绕“VLM 已经选中的目标”构建 selected-only 目标锁定链路：以 ByteTrack-style 两阶段检测关联为基础，叠加 VLM selected track 约束、类别约束、IoU 匹配、速度预测和 HSV 外观直方图相似度，实现从“语义选中目标”到“连续稳定目标框”的转换。

这里的“ByteTrack-style 追踪、selected-only 模式、HSV 外观特征匹配”不是三个割裂模块，而是同一个追踪器内部的多策略融合：

| 策略 | 解决的问题 | 具体作用 |
| --- | --- | --- |
| ByteTrack-style 两阶段关联 | 检测框置信度波动、短时漏检 | 高置信度检测先匹配，低置信度检测再补救，尽量维持轨迹连续 |
| selected-only 目标约束 | 多个同类物体抢目标 | VLM 选中目标后只输出 selected track，执行侧不会再被其他 YOLO 框干扰 |
| HSV 外观特征匹配 | 相似目标、遮挡后恢复、IoU 不稳定 | 记录选中目标区域颜色直方图，用外观相似度辅助判断是否还是原目标 |
| 类别约束 | 跨类别误关联 | 限制 bottle、box、cup、teddy bear 等不同类别之间错误绑定 |
| 速度预测 | 目标短时丢失 | 根据上一帧框位置变化预测短时位置，给重新匹配提供连续性 |
| 丢失缓冲与重选触发 | 目标离开视野或跟踪失败 | 短时丢失保留 track；持续丢失后由 lost_reselector 重新调用 VLM |

功能：

- 接收 YOLO detections。
- 接收 `/vlm_target_selector/selected_detection`。
- 将 VLM 选中的 detection 绑定为 selected track。
- 维护目标 `track_id`、`bbox`、`velocity`、`score`、`class_name`、`lost_frames` 等状态。
- 输出 `/object_tracker/selected_detection`。
- 发布状态 `/object_tracker/status`。

核心算法机制：

| 机制 | 说明 |
| --- | --- |
| 两阶段检测分组 | 将 YOLO detections 按置信度分为 `high_dets` 和 `low_dets` |
| 第一阶段匹配 | active tracks 与 high_dets 按 IoU 贪心匹配，阈值 `match_thresh` |
| 第二阶段补匹配 | 未匹配 track 再与 low_dets 匹配，阈值 `second_match_thresh`，提高低置信度场景下的连续性 |
| 类别约束 | `class_aware=true` 时只允许同类别目标关联，减少跨类别误跟踪 |
| 速度预测 | track 保存上一帧 bbox 变化量，丢失时用速度衰减预测短时位置 |
| VLM 目标锁定 | VLM selected_detection 与当前 tracks 按 IoU 绑定，得到 `selected_track_id` |
| 外观匹配 | 对选中目标区域计算 HSV 直方图，结合 IoU 与 appearance similarity 进行 selected track 更新 |
| 丢失缓冲 | 使用 `track_buffer` 允许目标短时丢失，超过阈值后清除 selected track |
| 丢失重选 | selected target 丢失约 1 秒后重新调用 VLM，8 秒仍未找回则判定当前画面没有目标 |

关键参数：

```text
track_high_thresh = 0.15
track_low_thresh = 0.02
new_track_thresh = 0.20
match_thresh = 0.7
second_match_thresh = 0.5
selected_match_thresh = 0.3
track_buffer = 45
class_aware = true
use_appearance_match = true
appearance_weight = 0.35
appearance_match_thresh = 0.45
```

作用：

- 避免 VLM 每帧重复调用。
- 在分段式 PID 视觉伺服对准过程中持续提供目标框。
- 降低云端 VLM 依赖和延迟。
- 在短时遮挡、检测置信度波动、目标轻微移动时维持 selected track 稳定。
- selected-only 输出让执行侧只看到“被确认的目标”，降低多个 YOLO 框同时存在时的误抓、误对准风险。
- 丢失重选机制让系统在目标被遮挡、机器人移动导致目标出框或检测短时失败时，可以重新让 VLM 选择目标，而不是直接失败。

### 4.9.1 YOLO-VLM-Tracking 融合目标选择与跟踪算法

系统不是简单地“YOLO 检测后直接抓取”，也不是每一帧都调用 VLM。实际采用的是多阶段闭环融合算法：

```text
YOLO 实时检测
  -> VLM 语义选框
  -> object_tracker 多策略 selected-only 持续跟踪
  -> lost_reselector 丢失后触发 VLM 重选
  -> 分段式 PID 视觉伺服使用稳定跟踪结果对准
```

该算法的输入包括：

- 当前 RGB 图像。
- YOLO 输出的候选检测框。
- 用户自然语言目标描述。
- 当前任务上下文，例如“最靠近卡皮巴拉的矿泉水”“放到矿泉水左边”。

第一阶段是 YOLO 候选生成。YOLO 在每帧图像中快速检测常见物品类别，输出候选框、类别和置信度。此阶段强调实时性和召回率，允许低置信度候选进入下一阶段，因为实际场景中瓶子、纸盒、玩具、包装袋等物品会受光照、遮挡、角度影响，单靠 YOLO 类别名无法稳定区分用户真正想要的目标。

第二阶段是 VLM 语义选择。系统将当前图像和 YOLO 候选框编号后交给 `qwen3-vl-plus`，让 VLM 根据用户目标描述选择最符合语义的候选编号。例如，当画面中有多个瓶子时，VLM 可以结合外观、相对位置和用户描述选择“纸盒左侧的农夫山泉”或“最靠近卡皮巴拉玩偶的矿泉水”。VLM 输出的不是控制指令，而是候选编号或目标确认结果。

第三阶段是 ByteTrack-style tracker 接管。VLM 只在目标选择或重新选择时调用一次或少量几次，选中目标后由 object_tracker 持续维护目标轨迹。tracker 将 YOLO detections 分为高置信度和低置信度两组，先用高置信度框更新 tracks，再用低置信度框补救未匹配 tracks；同时对 VLM 选中的目标启用 selected_track_id 锁定，并结合 IoU、类别、速度预测和 HSV 外观直方图相似度更新目标。进入 selected-only 状态后，执行链路只接收选中目标框，其他 YOLO 候选不会再参与底盘对准和机械臂抓取。这样可以避免每帧调用 VLM，降低云端延迟和费用，同时让分段式 PID 视觉伺服算法得到连续、稳定的目标位置。

第四个工程环节是丢失重选。若 selected target 短时丢失，tracker 先用 `track_buffer` 和速度预测维持目标；若选中目标持续丢失约 1 秒，lost_reselector 会使用当前任务目标描述重新调用 VLM，在最新画面中重新选择目标；若持续 8 秒仍未找回，则判定当前视野中没有目标，避免系统无限等待或反复调用 VLM。

融合算法输出：

```text
/object_tracker/selected_detection
```

随后 `selected_detection_bridge_node` 将该目标转发给执行侧：

```text
/tracked_yolov8/detections
```

分段式 PID 视觉伺服对准模块只消费跟踪后的目标框，不直接关心 VLM、YOLO 和 tracker 的内部细节。

该融合算法的优势：

- YOLO 保证实时性，适合持续视频流。
- VLM 保证语义准确性，适合处理复杂自然语言指代。
- 多策略 tracker 保证连续性和目标唯一性，适合底盘对准和机械臂抓取前的稳定目标锁定。
- VLM 不需要每帧调用，降低延迟、网络依赖和云端成本。
- 执行侧只接收稳定后的目标框，降低误抓和对准抖动风险。
- 支持目标丢失后的重新选择：选中目标丢失约 1 秒重新调用 VLM，8 秒仍未找回则明确判定没有物品。

算法伪代码：

```text
输入:
  image_t              当前图像
  detections_t         YOLO 当前帧候选框
  task_description     用户目标描述
  selected_track       当前跟踪目标，可为空

流程:
  if selected_track 不存在或 tracker 状态不稳定:
      candidates = filter(detections_t, target_classes, conf_threshold)
      numbered_image = draw_candidate_ids(image_t, candidates)
      selected_id = VLM_SELECT(numbered_image, candidates, task_description)
      if selected_id 有效:
          selected_detection = candidates[selected_id]
          tracker.create_or_lock_selected_track(selected_detection)
      else:
          返回 target_not_confirmed
  else:
      high_dets, low_dets = split_by_score(detections_t)
      tracks = greedy_iou_match(tracks, high_dets)
      tracks = second_stage_match(unmatched_tracks, low_dets)
      selected_detection = update_selected_track_by_iou_and_appearance()

  if selected_detection 稳定:
      publish /object_tracker/selected_detection
      bridge publish /tracked_yolov8/detections
      分段式 PID 视觉伺服 consume selected_detection
  else:
      进入重新选择或失败处理

输出:
  稳定目标框 selected_detection
```

### 4.10 selected_detection_bridge 视觉到执行桥接

`selected_detection_bridge_node` 将 object_tracker 的选中目标转发到执行侧：

```text
/object_tracker/selected_detection
  -> /tracked_yolov8/detections
```

分段式 PID 视觉伺服对准模块只需要订阅 `/tracked_yolov8/detections`，无需关心 VLM、YOLO、tracker 的内部实现。

创新点：

- 使用桥接节点隔离视觉智能系统和执行控制系统。
- 视觉算法可以独立快速迭代，执行侧接口保持稳定。

### 4.11 分段式 PID 视觉伺服对准模块

对准模块不是简单的单阶段视觉居中，而是一个融合视觉目标框、TOF 测距、激光雷达角度估计和 PID 控制的分段式视觉伺服状态机。该模块订阅 `/tracked_yolov8/detections` 中的稳定目标框，同时读取 `/laser` TOF 距离和 `/scan` 激光雷达数据，通过四阶段 PID 控制让麦克纳姆轮底盘逐步完成靠近、转正、贴近和横向像素级对准。

输入：

| 输入 | 作用 |
| --- | --- |
| `/tracked_yolov8/detections` | tracker 输出的稳定目标框，提供目标中心像素 `cx` |
| `/laser` Range | TOF 激光测距，提供机器人到目标/障碍的近距离距离 |
| `/scan` LaserScan | 激光雷达局部点云，用于拟合前方平面角度，估计偏航误差 |
| `/enable_redalign` | 总控启动/停止对准流程 |
| 参数服务器 | 动态设置抓取/放置的目标像素、距离、容差、最小速度等 |

输出：

| 输出 | 作用 |
| --- | --- |
| `/cmd_vel` | 控制麦克纳姆轮底盘前进、后退、横移和旋转 |
| `/red_align_success` | 对准完成信号，通知总控可以进入抓取或释放 |

控制器：

```text
pid_x   kp=0.012  kd=0.0015  控制前后距离
pid_y   kp=0.001  kd=0.005   控制横向像素误差
pid_yaw kp=0.02   kd=0.005   控制雷达拟合得到的偏航角误差
```

为避免低速死区导致底盘不动，系统对横移和旋转速度加入最小速度约束：

```text
min_vel_y = 0.05
min_vel_z = 0.10
```

为避免单帧抖动导致误判，每一阶段都使用滑动窗口稳态判断：

```text
window_size = 15
ratio_thresh = 0.4
```

即连续窗口内达到条件的比例超过阈值，才允许进入下一阶段。

四阶段状态机：

| 阶段 | 名称 | 目标 | 控制量 |
| --- | --- | --- | --- |
| 状态 1 | 第一段安全前进 | 根据 TOF 距离靠近到 stage1 距离，同时粗略横向对齐 | `linear.x` + `linear.y` |
| 状态 2 | 激光雷达平行转正 | 通过 `/scan` 局部点云拟合角度，使机器人与目标区域平面平行 | `angular.z` |
| 状态 3 | 贴脸距离精调 | 根据 TOF 距离微调到最终抓取/放置距离 | `linear.x` |
| 状态 4 | 像素级横向精调 | 根据目标框中心与目标像素误差进行横向微调 | `linear.y` |

保护逻辑：

- 如果 TOF 距离小于目标距离过多，立即后退恢复安全距离。
- 如果雷达 ROI 内有效点太少，暂停转正，等待有效雷达数据。
- 如果目标长时间丢失，停止底盘并回到等待状态。
- 每次启用对准时重置 PID、滑动窗口、滤波状态和成功标志。

普通抓取：

```text
target_pixel_x = 320
tol_pixel_fine = 10
target_dist_cm = 12.0
stage1_dist_cm = 16.0
```

相对放置：

```text
放到参照物左边：让参照物位于画面右侧，target_pixel_x = 480
放到参照物右边：让参照物位于画面左侧，target_pixel_x = 160
```

原因：

- 如果要把物体放到矿泉水左边，就需要机器人把夹持物对准矿泉水左侧的空间。
- 视觉上让矿泉水偏右，夹持物释放点就在画面中央附近，也就是矿泉水左侧。
- 反之，放到右边时让参照物偏左。

创新点：

- 不是单一视觉居中，而是“TOF 距离 + 雷达转正 + YOLO 像素误差”的分段式闭环。
- 每个阶段采用 PID 控制和滑动窗口稳态判定，降低单帧噪声导致的误触发。
- 相对放置不是瞄准参照物中心，而是把参照物移动到画面偏侧位置。
- 同一个分段式 PID 对准节点支持抓取和放置两套参数。
- 参数集中在代码中，便于现场按习惯调整。

### 4.12 深度相机与三维定位模块

深度图主要用于估计目标距离和三维位置。考虑 USB 带宽和 CPU 占用，系统支持将深度图降低到 320x240，而 RGB/YOLO 仍保持 640x480。

解决方案：

- YOLO 坐标基于 640x480。
- 深度图基于 320x240。
- 通过比例映射把 YOLO 中心点映射到深度图。
- 不只取单点深度，而是取目标中心周围小窗口，例如半径 `r=8` 左右。
- 对窗口内深度做过滤、排序、中值或合理值选择。

优势：

- 深度 USB 带宽下降。
- RGB 识别精度不下降。
- 多点深度统计比单点更抗噪。
- CPU 开销可控，窗口半径不宜过大。

创新点：

- 低分辨率深度与高分辨率 RGB 解耦。
- 用局部深度统计补偿低分辨率深度带来的误差。
- 兼顾 USB 带宽、CPU 占用和定位稳定性。

### 4.13 机械臂服务模块

机械臂由 `arm_node.py` 提供服务。

常用服务：

```text
/execute_grasp
/execute_release
/execute_retract
/execute_retract_140
```

抓取任务通常调用：

```text
/execute_grasp
```

相对放置任务还会调用：

```text
/execute_release
/execute_retract
```

记忆库找不到、需要现场搜索时，机械臂会调用：

```text
/execute_retract_140
```

作用是让机械臂进入不遮挡视野、便于搜索和对准的姿态。

夹爪优化：

- 不用固定最小检测角度判断是否夹到物体，避免大物体被误判。
- 施力角度使用真实反馈角度 + squeeze offset，而不是只相信目标角度。
- 等待舵机完成主要行程后再读取反馈，避免还没合上就误判。

创新点：

- 抓取闭合判断结合真实反馈，而不是只看下发目标。
- 对不同大小物体更稳，不依赖固定角度阈值。
- 搜索、抓取、释放、收回姿态分离，便于任务编排。

### 4.14 voice_fetch_orchestrator 总控模块

`voice_fetch_orchestrator.py` 是整个取物流程的执行总控。

职责：

- 接收 ASR 指令。
- 记录说话人位置。
- 调用 LLM 任务理解。
- 根据 intent 和 plan 执行任务。
- 查询物品记忆。
- 调用导航。
- 调用视觉确认。
- 调用分段式 PID 视觉伺服对准模块。
- 调用机械臂服务。
- 返回说话人位置。
- 播报状态。
- 记录任务摘要。

总控不是简单状态机，而是“计划执行器”。它根据 LLM 输出的结构化 plan 调用固定能力模块。

典型状态：

```text
idle
understanding
query_memory
navigate_to_object
confirm_target
track_target
pid_visual_servo_align
grasp
return_to_person
returned_to_person
release
retract
task_done
```

创新点：

- LLM 负责任务拆解，总控负责安全执行。
- 支持 plan context 缓存。比如第 4 步已经找到参照物，第 5 步相对放置会复用找到的目标，避免重复查询记忆和二次 VLM 失败。
- 支持任务摘要记忆，增强“刚才那个”“放回去”等自然交互。
- 对失败环节有明确状态和播报，便于日志定位。

## 5. 典型任务流程

### 5.1 普通取物任务

用户：

```text
帮我拿一下矿泉水
```

流程：

```text
ASR 识别文本
  -> LLM 输出 fetch_object
  -> 查询 object_memory.json
  -> 导航到矿泉水记忆点
  -> YOLO 检测 bottle/cup 等候选
  -> VLM 确认哪一个是矿泉水
  -> object_tracker 跟踪
  -> 分段式 PID 视觉伺服对准 target_pixel_x=320
  -> /execute_grasp
  -> 返回说话人位置
  -> 保持夹持结束
```

### 5.2 记忆库未命中但用户指定地点

用户：

```text
帮我拿卧室里的除锈剂
```

流程：

```text
LLM 提取 source_location=卧室 target=除锈剂
  -> 记忆库未命中
  -> 导航到卧室
  -> /execute_retract_140 机械臂进入搜索姿态
  -> 打开全类目 YOLO 搜索
  -> VLM 根据“除锈剂”选择候选
  -> 后续抓取流程继续
```

设计重点：

- 不能因为记忆库没有就直接失败。
- 用户给了地点，就应先去现场找。
- 现场 YOLO/VLM 仍找不到，才说明没有物品或看不到物品。

### 5.3 相对放置任务

用户：

```text
帮我把快递放到矿泉水左边
```

流程：

```text
LLM 输出 transfer/place_relative plan
  -> 找快递
  -> 抓取快递
  -> 找矿泉水
  -> 相对放置对准
     side=left
     target_pixel_x=480
  -> /execute_release
  -> /execute_retract
  -> 任务完成
```

重点：

- 放左边时不是瞄准矿泉水中心。
- 放左边时让矿泉水位于画面右侧。
- 放右边时让矿泉水位于画面左侧。

### 5.4 复杂跨房间任务

用户：

```text
帮我把卧室最靠近卡皮巴拉的农夫山泉
放到客厅靠近比比赞零食箱的维生素片左边
```

期望 plan：

```json
[
  {"action": "navigate_to", "target": "卧室"},
  {"action": "find_object", "target": "最靠近卡皮巴拉的农夫山泉"},
  {"action": "grasp_object", "target": "最靠近卡皮巴拉的农夫山泉"},
  {"action": "navigate_to", "target": "客厅"},
  {"action": "find_object", "target": "靠近比比赞零食箱的维生素片"},
  {"action": "place_relative", "reference": "靠近比比赞零食箱的维生素片", "side": "left"},
  {"action": "release_object"}
]
```

该流程体现了系统的多阶段理解能力：

- 识别源位置。
- 识别目标物。
- 理解目标物的限定描述。
- 识别目的位置。
- 识别放置参照物。
- 理解相对方向。
- 生成完整可执行计划。

## 6. LLM 编排机制

### 6.1 为什么需要 LLM

传统关键词规则难以覆盖自然语言表达。例如：

- “把这个放那边”
- “拿刚才那个”
- “最靠近卡皮巴拉的那个矿泉水”
- “双汇鸭舌箱旁边的柠檬茶右边”
- “回去吧”

LLM 适合做：

- 意图分类
- 任务拆解
- 上下文补全
- 名称归一
- 复杂限定语理解
- TTS 播报改写

但 LLM 不适合直接做：

- 底盘控制
- 实时避障
- 机械臂低层控制
- 目标坐标连续跟踪

所以系统采用“LLM 编排 + 传统控制执行”的架构。

### 6.2 安全边界

LLM 输出必须是结构化 JSON，总控只接受白名单动作：

```text
navigate_to
find_object
grasp_object
place_relative
release_object
return_to_person
chat
```

LLM 不能直接发布：

```text
/cmd_vel
/execute_grasp
/execute_release
```

所有真实动作由总控节点根据状态、服务可用性和视觉结果执行。

### 6.3 Prompt 设计原则

Prompt 主要约束：

- 当前指令优先，不被历史上下文覆盖。
- 相对放置必须生成显式 `place_relative` 步骤。
- top-level `placement_reference` 和 `placement_side` 只是兼容字段，不能替代 plan。
- 如果用户给出前置地点，记忆库失败也要现场寻找。
- 播报词可以有风格，但不能改变任务事实。
- 不允许编造已经完成的动作。

### 6.4 一次性 plan 一致性校验

LLM 输出 plan 后，系统不会立即无条件执行，而是先做一次轻量结构校验。该校验器不理解具体场景语义，也不替代 LLM 决策，只检查“LLM 自己输出的字段是否互相矛盾”。

例如，LLM 可能正确提取了：

```json
{
  "placement_reference": "维生素片",
  "placement_side": "left"
}
```

但 plan 中只写了：

```json
[
  {"action": "navigate_to", "target": "卧室"},
  {"action": "find_object", "target": "农夫山泉"},
  {"action": "grasp_object", "target": "农夫山泉"},
  {"action": "navigate_to", "target": "客厅"},
  {"action": "release_object"}
]
```

这类输出包含相对放置信息，却缺少 `place_relative` 步骤。校验器会把问题列表、原始用户指令和首次 JSON 一起交给同一个 LLM，请它只修复 plan 和明显不一致字段。

修复后期望结构类似：

```json
[
  {"action": "navigate_to", "target": "卧室"},
  {"action": "find_object", "target": "农夫山泉"},
  {"action": "grasp_object", "target": "农夫山泉"},
  {"action": "navigate_to", "target": "客厅"},
  {"action": "find_object", "target": "维生素片"},
  {"action": "place_relative", "reference": "维生素片", "side": "left"},
  {"action": "release_object"}
]
```

该机制只运行一次，不做多轮循环。这样可以兼顾稳定性和实时性：

- 避免 LLM 偶发漏步骤导致执行失败。
- 避免无限修复循环。
- 避免把规则写死到代码里，仍然由 LLM 理解“最靠近”“左边”“右边”等语义。
- 所有修复结果都会写入 `plan_validation` 元数据，便于后续日志审计。

### 6.5 上下文策略

系统只传最近 2 条任务摘要给 LLM，例如：

```json
[
  {
    "summary": "刚才从卧室拿起了农夫山泉矿泉水，当前仍保持夹持",
    "held_object": "农夫山泉矿泉水"
  },
  {
    "summary": "上一次任务把快递放到了矿泉水左边",
    "held_object": ""
  }
]
```

作用：

- 用户说“放回去”“刚才那个”时可以理解。
- 不传大量历史，降低延迟和误解。
- 避免 LLM 被旧任务干扰当前明确指令。

## 7. 创新点总结

### 7.1 App 一体化家庭服务入口

系统不是只做一个语音取物 demo，而是把 App 作为家庭服务机器人的一体化入口。App 一键启动负责底盘、建图、导航、巡逻、相机、视频流、状态显示和基础桥接；语音取物流程作为增量能力接入，不重复启动这些基础节点。

App 侧能力包括：

- 便捷建图：通过 App 切换建图模式，基于 Cartographer 完成环境地图构建。
- 导航：基于已保存地图进行定位和目标点导航，全局路径规划采用 Dijkstra，局部规划采用 DWA。
- 巡逻：按巡逻点位移动并扫描物品，生成物品空间记忆。
- 远程查看摄像头：局域网访问 `8081`，公网通过 frp 访问 `8888`。
- 电量查看：App 展示机器人当前电量和运行状态。
- TTS 播报风格切换：通过 `/voice_persona/set` 切换默认、管家、活泼、话痨或自定义风格。
- 远程语音控制：远程端可以发送语音或文本任务，触发取物、搬运、放置、返回等流程。

创新性在于：App 不是旁路调试工具，而是“Cartographer 建图-Dijkstra 全局规划-DWA 局部规划-巡逻-远程查看-状态监测-语音控制-播报风格”的统一用户界面；具身智能能力通过模块化补充脚本接入 App 主流程，既保留 App 易用性，又扩展了复杂自然语言任务执行能力。

### 7.2 视觉智能与执行控制解耦

系统将“看懂目标”和“执行动作”拆成两层。视觉智能层负责 YOLO 候选框、VLM 语义选择、ByteTrack-style 跟踪和任务状态输出；执行控制层负责导航、分段式 PID 视觉伺服、底盘速度、机械臂抓取/释放和安全停止。

具体边界：

- 视觉智能层输出 `/object_tracker/selected_detection`。
- 桥接层转发为 `/tracked_yolov8/detections`。
- 执行控制层只消费稳定目标框、TOF 距离和激光雷达数据。
- LLM/VLM 不直接发布 `/cmd_vel`，不直接操作机械臂。

创新性在于：云端大模型和本地控制器之间有清晰接口。视觉和大模型可以快速迭代，底盘与机械臂控制保持确定性和可验证性，降低大模型误输出对机器人运动安全的影响。

### 7.3 LLM 作为任务编排器

LLM 的角色是任务编排器，不是底层控制器。用户可以说“帮我把卧室的最靠近卡皮巴拉的农夫山泉放到客厅靠近比比赞零食箱的维生素片左边”，LLM 需要把这句话拆成地点、目标物、参照物、相对方向和可执行步骤。

编排结果采用受限 JSON：

- `intent`
- `target_name`
- `source_location`
- `destination`
- `placement_reference`
- `placement_side`
- `plan`
- `tts_text`

创新性在于：自然语言中的“最靠近某物”“放到某物左边”“回去吧”“刚才那个”等表达，被转换成可检查、可回放、可校验的结构化计划。执行侧只支持白名单动作，例如导航、查找、抓取、相对放置、释放和返回，避免 LLM 直接控制底盘或机械臂。

### 7.4 物品记忆与现场搜索结合

系统把“巡逻记忆库”和“现场视觉搜索”结合起来。巡逻时生成 `object_memory.json`，记录物品名称、可能名称、YOLO 类别、巡逻点和地图位置；执行任务时优先查记忆，提高效率。

当记忆库未命中但用户指定了前置地点，例如“卧室的矿泉水”，系统不会直接失败，而是导航到卧室后打开全类别 YOLO 检测，再用 VLM 在现场候选中确认目标。

创新性在于：记忆库是加速器，不是硬约束。家庭环境中的物品经常被移动或新增，系统允许“记忆命中快速执行、记忆失败现场搜索、搜索失败再明确播报没有找到”，比单纯依赖固定数据库更适合真实场景。

### 7.5 YOLO-VLM-Tracking 多阶段闭环融合

系统采用 YOLO-VLM-Tracking 多策略融合算法，而不是单独依赖 YOLO 类别名或每帧调用 VLM。该算法的核心是：YOLO 负责实时召回，VLM 负责语义选中，object_tracker 负责 selected-only 持续锁定，lost_reselector 负责目标丢失后的语义重选。

主要阶段如下：

1. YOLO 实时检测：RDK X5 BPU 上运行 `.bin` 检测模型，输出候选框、类别和置信度。
2. VLM 语义选框：`qwen3-vl-plus` 根据用户描述和带编号候选图选择目标编号。
3. 多策略 tracker 持续锁定：选中后不再每帧调用 VLM，而是用 ByteTrack-style 两阶段关联、selected-only 约束、IoU、类别约束、速度预测和 HSV 外观直方图维持 selected track。
4. 丢失重选：selected target 丢失约 1 秒后重新调用 VLM，8 秒仍未找回则判定当前画面没有目标。

追踪算法细节：

- 高低置信度两阶段关联：`high_dets` 先匹配，`low_dets` 补救低置信度目标。
- IoU 贪心匹配：用 bbox 重叠程度维持轨迹连续。
- 类别约束：避免瓶子、盒子、玩具之间互相误关联。
- 速度预测：目标短时丢失时用上一帧运动趋势预测位置。
- 外观相似度：对 VLM 选中目标计算 HSV 直方图，辅助 selected track 更新。
- 丢失缓冲：短时遮挡不会立即清除目标。
- selected-only 输出：VLM 选中目标后，执行侧只消费 selected track，其他 YOLO 框不会进入分段式 PID 对准。
- 丢失重选：目标短时丢失时先由 tracker 缓冲，持续丢失后再触发 VLM 重选，避免每帧调用 VLM。

创新性在于：YOLO 负责实时召回，VLM 负责语义判别，ByteTrack-style 关联保证连续性，selected-only 约束保证执行目标唯一，HSV 外观特征提升相似物体场景下的稳定性，lost_reselector 让目标丢失后可以重新语义确认。这样既能理解“最靠近卡皮巴拉的矿泉水”这类复杂描述，又能在底盘移动和机械臂抓取前保持稳定目标框，避免 VLM 每帧调用导致的高延迟和高成本。

### 7.6 相对放置的视觉偏置对准

相对放置不是瞄准参照物中心，而是通过目标像素偏置实现“放到左边/右边”。例如：

```text
放到矿泉水左边：让矿泉水位于画面右侧，target_pixel_x = 480
放到矿泉水右边：让矿泉水位于画面左侧，target_pixel_x = 160
```

这样机器人夹持物的释放区域会落在参照物另一侧。普通抓取使用 `target_pixel_x = 320` 和更严格的 `tol_pixel_fine = 10`，相对放置使用偏置目标像素和独立容差。

创新性在于：把自然语言空间关系转成图像坐标系中的可调参数，在不引入复杂三维放置规划的情况下实现实用的左右相对放置，同时复用同一个分段式 PID 视觉伺服控制器。

### 7.7 分段式 PID 视觉伺服对准

对准算法不是简单“让框居中”，而是分四段执行：

1. TOF 距离安全靠近，同时粗横向对齐。
2. 激光雷达局部点云拟合平面，旋转到与目标区域平行。
3. TOF 距离精调到抓取/放置距离。
4. 根据目标框中心像素做横向微调。

控制器包含三组 PID：

```text
pid_x   控制前后距离
pid_y   控制横向像素误差
pid_yaw 控制雷达拟合偏航角
```

每一阶段使用滑动窗口判断稳定状态，避免单帧检测抖动导致误完成；同时具备目标丢失停止、距离过近后退、雷达无效等待等保护逻辑。

创新性在于：把 TOF、激光雷达、YOLO/tracker 目标框和麦克纳姆轮全向运动统一到一个分段式视觉伺服状态机中。它既支持普通抓取居中，也支持相对放置偏置对准，是视觉识别到机械臂动作之间的关键执行闭环。

### 7.8 低分辨率深度 + 局部统计

深度图降到 320x240，RGB/YOLO 仍保持 640x480。

实现方式：

- YOLO 坐标基于 RGB 图像。
- 按比例映射到低分辨率深度图。
- 不取单点深度，而取目标中心周围窗口。
- 对窗口内深度做过滤、中值或合理值选择。

创新性在于：识别和深度解耦。RGB 保持较高分辨率保证检测效果，深度降低分辨率减少 USB 带宽和 CPU 压力，再用局部统计弥补深度噪声。

### 7.9 播报风格可切换

机器人“小瓜”支持多种 persona。

机制：

- App 通过 `/voice_persona/set` 切换风格。
- 常见状态使用固定短句保证稳定。
- 需要娱乐性和变化时调用 `qwen3.6-flash` 生成播报文本。
- 最终用 `qwen3-tts-instruct-flash-realtime` 合成语音。
- 自定义风格切换成功后，可生成一句符合自定义风格的确认播报。

创新性在于：把“说什么”和“怎么说”分开。任务事实由总控确定，语言风格由 persona 控制，既能保持任务播报准确，又能提供话痨、活泼、管家等娱乐化体验。

### 7.10 任务摘要记忆

系统记录最近任务摘要，而不是全量对话。

策略：

- 只保留最近 2 条任务摘要。
- 记录是否仍夹持物品。
- 只在用户使用“刚才那个”“放回去”“继续”等省略表达时提供帮助。

创新性在于：用轻量任务记忆解决上下文问题，不把长对话全部发送给 LLM，降低 token、延迟和旧信息干扰。

### 7.11 服务器与远程通信集成

系统构建了机器人本地服务和公网服务器之间的远程通信链路：

```text
RDK X5 本地 MJPEG 8081
  -> frpc
  -> frps public-server.example:1935
  -> 公网视频 public-server.example:8888
```

同时保留 MQTT TCP `1883`、MQTT WebSocket `9001`、App rosbridge `9090`、语音 rosbridge `9091` 和速度控制 WebSocket `19091`。

创新性在于：远程访问、App 控制、语音取物和底层执行链路被端口隔离，公网服务器只做通信入口，不承载机器人核心控制。即使远程视频或云端网络波动，机器人本地导航、底盘控制和机械臂执行仍保持本地闭环。

### 7.12 工程级稳定性优化

包括：

- 禁用 USB autosuspend。
- XVF3800 PCM 健康检查。
- Qwen TTS WebSocket 连接超时从 5 秒提升到 10 秒。
- camera_mux 高 CPU 优化。
- 空闲节点降 CPU。
- 深度图降分辨率。
- 不同功能端口隔离。
- 日志分文件保存。
- 节点状态可观测。

## 8. 可靠性与故障处理

### 8.1 USB 掉线

现象：

- XVF3800 唤醒失败。
- 串口掉线。
- hub 反复 disconnect/reconnect。

处理：

- 禁用 USB autosuspend。
- 启动时重枚举 XVF3800。
- PCM health 检查 zero-filled。
- 建议关键串口不经过无源 hub。

### 8.2 TTS 网络不稳定

现象：

```text
websocket connection could not established within 5s
Qwen TTS timeout waiting for audio
Network is unreachable
```

处理：

- Qwen TTS 连接等待提升到 10 秒。
- 固定播报可以作为 fallback。
- 自定义风格 LLM 生成失败时使用固定“自定义风格已切换”。

### 8.3 VLM 目标选择失败

现象：

- VLM 没选出编号。
- 选错相似物体。
- 候选框编号解析错误。

处理：

- Prompt 限制只输出最终编号。
- 解析时优先结论数字。
- 保存调试图片。
- 检查 YOLO 候选是否包含目标。

### 8.4 机械臂抓取不稳定

现象：

- 夹爪还没闭合就判断结束。
- 负载变高时舵机变慢。
- 实际角度与目标角度不一致。

处理：

- 等舵机完成主要行程再读取反馈。
- 使用真实反馈角度 + squeeze offset。
- 避免固定最小角度阈值。

### 8.5 导航异常

可能原因：

- USB hub 抖动导致底盘串口掉线。
- `/cmd_vel` 被多个来源覆盖。
- App 手动控制和语音任务同时操作。

处理：

- 任务执行时不要 App 手动遥控。
- 不要切换建图/导航/巡逻模式。
- 检查 USB 拓扑和供电。
- 底盘 driver_node 应具备串口重连能力。

## 9. 部署与日志

### 9.1 启动顺序

先启动 App：

```bash
cd /opt/xiaogua/ros2_ws
bash src/robopilot_app_bridge/scripts/start_robopilot_app.sh
```

再启动语音取物：

```bash
cd /opt/xiaogua/ros2_ws
bash src/robopilot_app_bridge/scripts/start_voice_fetch_autoaim.sh
```

### 9.2 关键日志

```text
/tmp/voice_fetch_autoaim_logs/ros1_voice_fetch_orchestrator.log
/tmp/voice_fetch_autoaim_logs/ros1_auto_aim.log
/tmp/voice_fetch_autoaim_logs/ros1_arm_node.log
/tmp/voice_fetch_autoaim_logs/ros1_rosbridge.log
/tmp/voice_fetch_autoaim_logs/selected_detection_bridge.log
/tmp/voice_fetch_autoaim_logs/fetch_task_bridge.log
/tmp/voice_fetch_autoaim_logs/respeaker.log
/tmp/voice_fetch_autoaim_logs/tts_host.log
/tmp/voice_fetch_autoaim_logs/persona_control.log
```

### 9.3 常用观察命令

ROS 执行侧：

```bash
rostopic echo /asr_command
rostopic echo /voice_fetch/state
rostopic echo /voice_fetch/person_pose_map
rostopic echo /tracked_yolov8/detections
rostopic echo /red_align_success
```

ROS 智能侧：

```bash
ros2 topic echo /task_understanding/result
ros2 topic echo /object_memory/query_result
ros2 topic echo /target_confirm/result
ros2 topic echo /memory_target_selector/result
ros2 topic echo /object_tracker/status
```

## 10. 系统优势

本系统的优势可以概括为：

- 能听懂自然语言，而不只是固定命令词。
- 能把复杂语音任务拆成结构化 plan，并限制在白名单动作内执行。
- 能利用巡逻记忆，提高找物效率。
- 记忆失败时仍可现场搜索。
- 能处理复杂限定语和相对放置。
- 能通过 YOLO-VLM-Tracking 多策略融合实现语义目标锁定。
- 能通过 TOF、激光雷达、目标框和 PID 组成的分段式视觉伺服完成精对准。
- 能在既有机器人控制链路上接入视觉智能能力，而不破坏底层执行稳定性。
- 能与 App 一键启动共存。
- App 支持便捷建图、导航、巡逻、远程视频、电量查看、TTS 风格切换和远程语音控制。
- 公网服务器和 frpc/frps 提供远程视频入口，本地控制链路仍在 RDK X5 闭环运行。
- 有多风格 TTS，提升交互趣味性。
- 有明确日志和状态，方便定位问题。
- 通过工程优化控制 CPU、USB 和网络风险。

## 10.1 建议评估指标

为了让系统能力可量化，建议从任务成功率、识别准确性、实时性和资源占用四个维度评估。

| 指标 | 含义 | 观察方式 |
| --- | --- | --- |
| ASR 成功率 | 唤醒后正确识别用户指令的比例 | `/asr_command` 与人工标注对比 |
| LLM plan 正确率 | 结构化 plan 是否包含必要步骤 | `/task_understanding/result` 日志审查 |
| 记忆命中率 | 物品能否从 object_memory.json 找到有效点位 | `/object_memory/query_result` |
| 现场搜索成功率 | 记忆失败后 YOLO/VLM 能否现场找到目标 | target_confirm 与 selector 日志 |
| VLM 选框准确率 | VLM 选择的编号是否为真实目标 | VLM 调试图人工复核 |
| tracker 稳定率 | 目标跟踪是否持续稳定 | `/object_tracker/status` |
| 分段式 PID 视觉伺服成功率 | 是否成功输出 `/red_align_success` | 对准节点日志 |
| 抓取成功率 | `/execute_grasp` 后是否真实夹住物品 | arm_node 日志与人工观察 |
| 相对放置成功率 | 左/右放置是否符合用户语义 | 任务后图像或人工确认 |
| 端到端任务成功率 | 从语音到任务完成的整体成功率 | voice_fetch_orchestrator 状态 |
| 平均任务耗时 | 单次任务从接收指令到结束的时间 | 总控日志时间戳 |
| CPU/USB 占用 | 是否出现满载、排队或掉线 | top、dmesg、节点日志 |

## 11. 后续优化方向

### 11.1 更强的任务计划语义校验

当前系统已经实现一次性 plan 一致性校验，可检查相对放置、导航、抓取和释放等结构步骤是否缺失。后续可以在不破坏模块化边界的前提下继续增强语义级校验：

- 检查“最靠近某物”的限定语是否被保留到目标选择阶段。
- 检查 source location、destination location、target 和 reference 是否被 plan 正确传递。
- 对明显矛盾的结果给出人工可读日志，例如“字段中有相对放置，但 plan 中没有相对放置动作”。
- 在不循环重试的前提下，必要时触发一次 LLM 重新修复。

该方向仍然坚持“校验结构，不写死具体任务规则”，避免把自然语言理解退化成大量硬编码 if-else。

### 11.2 VLM 结果自检

VLM 选中目标后，可让 VLM 输出简短理由或目标属性，用于日志审计，但执行接口仍只使用编号。

### 11.3 本地 fallback TTS

当 Qwen TTS 网络失败时，可以使用本地 TTS 或预录音短句，保证关键状态一定能播报。

### 11.4 更完善的放置策略

当前相对放置主要依赖图像横向偏置和距离控制。后续可以结合深度三维坐标和桌面平面估计，提高放置精度。

### 11.5 更稳定的 USB 硬件拓扑

建议长期使用带外接供电的 USB hub，并将底盘串口、机械臂串口、相机和麦克风分散到更稳定的 USB 路径。

## 12. 结论

基于 RDK X5 的端云协同多模态具身智能家庭服务机器人“小瓜”不是单一模型或单一节点，而是一个完整的多模态机器人任务执行系统。它将 App 便捷建图、导航、巡逻、远程视频、电量查看、TTS 风格切换和远程语音控制，与本地 ROS 导航、机械臂执行、YOLO BPU 推理、VLM 目标确认、多策略目标追踪、分段式 PID 视觉伺服对准和任务记忆整合在一起。

系统的核心创新在于：用 `qwen3.6-flash` 将自然语言任务转成受限结构化 plan，用 `qwen3-vl-plus` 在 YOLO 候选框中完成语义目标选择，用 ByteTrack-style 跟踪算法维持目标连续锁定，再由 TOF、激光雷达、目标框和 PID 组成的分段式视觉伺服状态机驱动麦克纳姆轮底盘完成抓取或相对放置前的精对准。公网服务器、frpc/frps、MQTT、rosbridge 和本地视频服务提供远程入口，但机器人核心运动控制仍在 RDK X5 本地闭环执行。

这种架构把云端大模型的语义理解能力、本地 BPU 视觉推理能力、传统机器人控制的确定性和 App 的易用入口结合起来，适合真实家庭或实验室服务机器人场景持续迭代。
