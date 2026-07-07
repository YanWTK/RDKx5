#!/usr/bin/env python3
"""ROS2-ROS1 桥接节点：ASR转发 + TTS转发 + 唤醒回复"""

import json
import os
import threading
import time
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    exit(1)


class Bridge(Node):
    def __init__(self):
        super().__init__('asr_to_ros1_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 5.0)
        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        self.ws = None
        self.ws_lock = threading.RLock()  # RLock to avoid deadlock in nested calls
        self.connected = False
        self._tts_playing = False
        self._tts_lock = threading.Lock()
        self._kws_enabled = None
        self._kws_lock = threading.Lock()
        self._wake_session_lock = threading.Lock()
        self._wake_waiting_for_task = False
        self._wake_task_started = False
        self._wake_session_id = 0
        self._recording_active = False

        # ROS2 订阅
        self.create_subscription(String, '/xvf3800/asr/result', self._on_asr, 10)
        self.create_subscription(Bool, '/xvf3800/wake_detected', self._on_wake, 10)
        self.create_subscription(
            Bool, '/xvf3800/asr/recording', self._on_recording, 10
        )
        self.create_subscription(Bool, '/tts_playing', self._on_tts_playing, 10)
        self.create_subscription(String, '/voice_persona/set', self._on_voice_persona_set, 10)

        # ROS2 TTS发布
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)
        kws_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.kws_control_pub = self.create_publisher(
            Bool, '/xvf3800/kws/enabled', kws_qos
        )

        # ROS2 ASR服务
        self.record_client = self.create_client(Trigger, '/xvf3800/record_asr')
        self.get_logger().info('等待 record_asr 服务...')
        self.record_client.wait_for_service()
        self.get_logger().info('record_asr 就绪')

        # ROS1 TTS订阅状态
        self._ros1_tts_subscribed = False
        self._ros1_state_subscribed = False

        # 唤醒回复词
        self._persona_path = os.getenv(
            'VOICE_PERSONA_PATH',
            '/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json',
        )
        self._persona_profile_name = os.getenv('VOICE_PERSONA_PROFILE', '').strip()
        self._wake_replies_mtime = None
        self._wake_replies = self._load_wake_replies()
        self._last_wake_reply = None

        self.get_logger().info(f'桥接启动 url={self.url}')
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def _load_wake_replies(self):
        default_replies = [
            '__local__:wake_reply_zaine.wav',
            '我在，什么任务',
            '来了来了，说吧',
            '收到信号，请下指令',
            '小瓜上线，请安排',
            '我听着呢',
            '在，准备干活',
            '老板请讲',
            '雷达已竖起来了',
            '我在，别太难就行',
            '收到，今天也很靠谱',
            '小瓜待命中',
        ]

        env_replies = os.getenv('WAKE_REPLIES', '').strip()
        if env_replies:
            try:
                parsed = json.loads(env_replies)
                if isinstance(parsed, list):
                    replies = [str(item).strip() for item in parsed if str(item).strip()]
                else:
                    replies = [part.strip() for part in env_replies.split('|') if part.strip()]
            except Exception:
                replies = [part.strip() for part in env_replies.split('|') if part.strip()]
            if replies:
                return replies

        try:
            self._wake_replies_mtime = os.path.getmtime(self._persona_path)
            with open(self._persona_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            profiles = data.get('profiles', {}) if isinstance(data, dict) else {}
            active_name = self._persona_profile_name or str(data.get('active_profile') or 'default')
            profile = profiles.get(active_name) or profiles.get('default') or {}
            replies = profile.get('wake_replies') if isinstance(profile, dict) else None
            if isinstance(replies, list):
                replies = [str(item).strip() for item in replies if str(item).strip()]
                if replies:
                    return replies
        except Exception as exc:
            self.get_logger().warning(f'读取唤醒回复配置失败，使用默认回复: {exc}')
        return default_replies

    def _refresh_wake_replies(self):
        if os.getenv('WAKE_REPLIES', '').strip():
            return
        try:
            mtime = os.path.getmtime(self._persona_path)
        except OSError:
            return
        if self._wake_replies_mtime == mtime:
            return
        self._wake_replies = self._load_wake_replies()

    def _choose_wake_reply(self):
        self._refresh_wake_replies()
        if not self._wake_replies:
            return '__local__:wake_reply_zaine.wav'
        if len(self._wake_replies) == 1:
            reply = self._wake_replies[0]
        else:
            candidates = [
                item for item in self._wake_replies if item != self._last_wake_reply
            ]
            reply = random.choice(candidates or self._wake_replies)
        self._last_wake_reply = reply
        return reply

    def _connect_loop(self):
        while rclpy.ok():
            if not self.connected:
                self._try_connect()
            time.sleep(self.reconnect_interval)

    def _try_connect(self):
        try:
            with self.ws_lock:
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
                self.ws = websocket.WebSocket()
                self.ws.settimeout(5)
                self.ws.connect(self.url)
                self.connected = True
                self._ros1_tts_subscribed = False
                self._ros1_state_subscribed = False
                self._subscribe_ros1_topics()
                self.get_logger().info(f'已连接 rosbridge: {self.url}')
        except Exception as e:
            self.get_logger().warn(f'连接失败: {e}')
            self.connected = False

    def _subscribe_ros1_topics(self):
        """Subscribe to TTS and the latched fetch state over rosbridge."""
        try:
            with self.ws_lock:
                if self.ws and self.connected:
                    if not self._ros1_tts_subscribed:
                        self.ws.send(json.dumps({
                            'op': 'subscribe',
                            'topic': '/tts_text',
                            'type': 'std_msgs/String',
                        }))
                        self._ros1_tts_subscribed = True
                    if not self._ros1_state_subscribed:
                        self.ws.send(json.dumps({
                            'op': 'subscribe',
                            'topic': '/voice_fetch/state',
                            'type': 'std_msgs/String',
                        }))
                        self._ros1_state_subscribed = True
                    self.get_logger().info('已订阅ROS1 /tts_text 和 /voice_fetch/state')
        except Exception:
            pass

    def _send_ros1(self, topic, data):
        """发送消息到ROS1话题"""
        try:
            with self.ws_lock:
                if self.ws and self.connected:
                    advertise = json.dumps({'op': 'advertise', 'topic': topic, 'type': 'std_msgs/String'})
                    self.ws.send(advertise)
                    publish = json.dumps({'op': 'publish', 'topic': topic, 'msg': data})
                    self.ws.send(publish)
                    return True
        except Exception:
                self.connected = False
        return False

    def _on_tts_playing(self, msg):
        with self._tts_lock:
            self._tts_playing = bool(msg.data)

    def _on_voice_persona_set(self, msg):
        if not str(msg.data or '').strip():
            return
        if not self._send_ros1('/voice_persona/set', {'data': msg.data}):
            self.get_logger().warning('转发 voice_persona 设置到 ROS1 失败')

    def _is_tts_playing(self):
        with self._tts_lock:
            return self._tts_playing

    def _set_kws_enabled(self, enabled, reason):
        enabled = bool(enabled)
        with self._kws_lock:
            if self._kws_enabled == enabled:
                return
            self._kws_enabled = enabled
        self.kws_control_pub.publish(Bool(data=enabled))
        self.get_logger().info(
            f"KWS {'开启' if enabled else '暂停'}: {reason}"
        )

    def _start_wake_session(self):
        with self._wake_session_lock:
            self._wake_session_id += 1
            self._wake_waiting_for_task = True
            self._wake_task_started = False
            self._recording_active = False
            session_id = self._wake_session_id
        self._send_doa_session('wake', session_id)
        return session_id

    def _abort_wake_session(self, reason):
        with self._wake_session_lock:
            session_id = self._wake_session_id
            self._wake_waiting_for_task = False
            self._wake_task_started = False
            self._recording_active = False
        self._send_doa_session('abort', session_id)
        self._set_kws_enabled(True, reason)

    def _send_doa_session(self, event, session_id):
        payload = json.dumps({
            'event': str(event),
            'session_id': int(session_id),
            'timestamp': time.time(),
        }, ensure_ascii=False)
        if not self._send_ros1('/voice_fetch/doa_session', {'data': payload}):
            self.get_logger().warning(
                f'DOA会话事件转发失败: event={event} session={session_id}'
            )

    def _on_recording(self, msg):
        active = bool(msg.data)
        with self._wake_session_lock:
            if not self._wake_waiting_for_task:
                return
            if self._recording_active == active:
                return
            self._recording_active = active
            session_id = self._wake_session_id
        self._send_doa_session(
            'recording_start' if active else 'recording_end', session_id
        )

    def _on_fetch_state(self, state):
        state = str(state).strip()
        if not state:
            return
        if state != 'idle':
            with self._wake_session_lock:
                if self._wake_waiting_for_task:
                    self._wake_task_started = True
            self._set_kws_enabled(False, f'voice_fetch/state={state}')
            return

        with self._wake_session_lock:
            if self._wake_waiting_for_task and not self._wake_task_started:
                self.get_logger().info('忽略唤醒会话期间迟到的 idle 状态')
                return
            self._wake_waiting_for_task = False
            self._wake_task_started = False
        self._set_kws_enabled(True, 'voice_fetch/state=idle')

    def _on_wake(self, msg):
        if not msg.data:
            return
        if self._is_tts_playing():
            self.get_logger().info('TTS播放中，忽略唤醒事件')
            self._abort_wake_session('唤醒事件被TTS抑制')
            return
        self._start_wake_session()
        self._set_kws_enabled(False, '已唤醒，等待录音和任务结束')
        reply = self._choose_wake_reply()
        self.get_logger().info(f'唤醒 → 播放回复: {reply}')
        # 直接发布到ROS2 /tts_text（宿主机TTS节点播放）
        tts_msg = String()
        tts_msg.data = reply
        self.tts_pub.publish(tts_msg)
        # 0.5秒后触发ASR
        threading.Thread(target=self._delayed_record, daemon=True).start()

    def _wait_tts_idle(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if not self._is_tts_playing():
                return True
            time.sleep(0.05)
        return not self._is_tts_playing()

    def _delayed_record(self):
        # Give tts_host_node a moment to publish /tts_playing=true, then wait
        # for playback to finish so speaker leakage does not trigger ASR.
        time.sleep(0.1)
        if not self._wait_tts_idle(timeout_sec=5.0):
            self.get_logger().warning('TTS播放状态超时，跳过本次ASR录音')
            self._abort_wake_session('TTS超时，未开始录音')
            return
        time.sleep(0.2)
        self.get_logger().info('触发ASR录音...')
        req = Trigger.Request()
        future = self.record_client.call_async(req)
        future.add_done_callback(self._on_record_done)

    def _on_record_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'ASR录音服务异常: {exc}')
            self._abort_wake_session('ASR录音服务异常')
            return
        if response is None or not response.success:
            reason = response.message if response is not None else 'empty response'
            self.get_logger().info(f'本次录音没有有效指令: {reason}')
            self._abort_wake_session('录音结束且没有有效语音')
            return
        try:
            text = str(json.loads(response.message).get('text', '')).strip()
        except Exception:
            text = ''
        if not text:
            self._abort_wake_session('录音结束且ASR文本为空')

    def _on_asr(self, msg):
        """收到ASR结果，转发到ROS1"""
        if self._is_tts_playing():
            self.get_logger().info('TTS播放中，忽略ASR结果')
            self._abort_wake_session('TTS播放期间忽略ASR结果')
            return
        self.get_logger().info(f'ASR结果: {msg.data[:80]}')
        try:
            data = json.loads(msg.data)
            text = data.get('text', '')
            if text:
                if self._send_ros1('/asr_command', {'data': text}):
                    self.get_logger().info(f'→ ROS1: {text}')
                else:
                    self.get_logger().warning('ASR结果无法发送到ROS1，恢复KWS')
                    self._abort_wake_session('ROS1转发失败')
            else:
                self._abort_wake_session('ASR文本为空')
        except Exception:
            self._abort_wake_session('ASR结果解析失败')

    def _poll_ros1_tts(self):
        """轮询ROS1 /tts_text消息，转发到ROS2"""
        while rclpy.ok():
            if self.connected:
                try:
                    with self.ws_lock:
                        if self.ws and self.connected:
                            self.ws.settimeout(0.1)
                            try:
                                raw = self.ws.recv()
                                if raw:
                                    msg = json.loads(raw)
                                    if msg.get('op') == 'publish':
                                        topic = msg.get('topic')
                                        text = msg.get('msg', {}).get('data', '')
                                        if topic == '/tts_text':
                                            if text and not text.startswith('__local__:'):
                                                tts_msg = String()
                                                tts_msg.data = text
                                                self.tts_pub.publish(tts_msg)
                                                self.get_logger().info(f'ROS1 TTS → ROS2: {text}')
                                        elif topic == '/voice_fetch/state':
                                            state = str(text).strip()
                                            if state:
                                                self._on_fetch_state(state)
                            except websocket.WebSocketTimeoutException:
                                pass
                            except Exception:
                                pass
                except Exception:
                    self.connected = False
            time.sleep(0.1)

    def destroy_node(self):
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Bridge()
    threading.Thread(target=node._poll_ros1_tts, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
