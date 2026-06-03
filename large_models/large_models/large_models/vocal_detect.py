#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import base64
import json
import math
import os
import subprocess
import time
import rclpy
import threading
import gc
import tempfile
import urllib.error
import urllib.request
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import SetBool, Trigger, Empty

from speech import awake
from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetInt32

class VocalDetect(Node):
    """

    Voice detection node class, responsible for voice wake-up and speech recognition(语音检测节点类，负责语音唤醒和语音识别)
    """
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True  # Running flag (运行标志)

        # Declare parameters(声明参数)
        self.declare_parameter('awake_method', 'xf')
        self.declare_parameter('mic_type', 'mic6_circle')
        self.declare_parameter('port', '/dev/ring_mic')
        self.declare_parameter('enable_wakeup', True)
        self.declare_parameter('enable_setting', False)
        self.declare_parameter('awake_word', 'hello hi wonder')
        self.declare_parameter('asr_provider', os.getenv('ASR_PROVIDER', 'dashscope'))
        self.declare_parameter('asr_channels', 1)
        self.declare_parameter('asr_input_device_index', -1)
        self.declare_parameter('stepfun_asr_model', os.getenv('STEPFUN_ASR_MODEL', 'stepaudio-2.5-asr'))
        self.declare_parameter('stepfun_asr_endpoint', os.getenv('STEPFUN_ASR_ENDPOINT', 'https://api.stepfun.com/v1/audio/asr/sse'))
        self.declare_parameter('stepfun_asr_device', os.getenv('STEPFUN_ASR_DEVICE', 'default'))
        self.declare_parameter('stepfun_record_seconds', float(os.getenv('STEPFUN_RECORD_SECONDS', '4.0')))
        self.declare_parameter('mode', 1)
        self.declare_parameter('offline', 'false')
        self.declare_parameter('punct_model', '')

        self.awake_method = self.get_parameter('awake_method').value  # Wake-up method (唤醒方法)
        mic_type = self.get_parameter('mic_type').value  # Microphone type (麦克风类型)
        port = self.get_parameter('port').value  # Device port (设备端口)
        awake_word = self.get_parameter('awake_word').value  # Wake-up word (唤醒词)
        self.asr_provider = str(self.get_parameter('asr_provider').value).strip().lower()
        self.asr_channels = int(self.get_parameter('asr_channels').value)
        self.asr_input_device_index = int(self.get_parameter('asr_input_device_index').value)
        self.stepfun_asr_model = str(self.get_parameter('stepfun_asr_model').value).strip()
        self.stepfun_asr_endpoint = str(self.get_parameter('stepfun_asr_endpoint').value).strip()
        self.stepfun_asr_device = str(self.get_parameter('stepfun_asr_device').value).strip()
        self.stepfun_record_seconds = max(1.0, float(self.get_parameter('stepfun_record_seconds').value))
        self.stepfun_api_key = (
            os.getenv('STEPFUN_API_KEY')
            or os.getenv('STEP_API_KEY')
            or os.getenv('STEPFUN_KEY')
            or stepfun_api_key
        )
        enable_setting = self.get_parameter('enable_setting').value  # Enable settings (启用设置)
        self.enable_wakeup = self.get_parameter('enable_wakeup').value  # Enable wake-up (启用唤醒)
        self.mode = int(self.get_parameter('mode').value)  # Working mode (工作模式)
        self.punct_model = self.get_parameter('punct_model').value
        self.sherpa_onnx = None
        if os.environ["ASR_MODE"] == 'offline':
            import sherpa_onnx
            self.offline = True
            self.sherpa_onnx = sherpa_onnx
        else:
            self.offline = False
            if os.environ["ASR_LANGUAGE"] == 'Chinese' and self.asr_provider in ('dashscope', 'aliyun') and not api_key:
                self.get_logger().warn(
                    'ALIYUN_API_KEY/DASHSCOPE_API_KEY is empty; online Chinese ASR will fail'
                )
            if os.environ["ASR_LANGUAGE"] == 'Chinese' and self.asr_provider == 'stepfun' and not self.stepfun_api_key:
                self.get_logger().warn(
                    'STEPFUN_API_KEY/STEP_API_KEY is empty; StepFun online ASR will fail'
                )

        # Initialize wake-up device (初始化唤醒设备)
        if self.awake_method == 'xf':
            self.kws = awake.CircleMic(port, awake_word, mic_type, enable_setting)
        else:
            self.kws = awake.WonderEchoPro(port) 
        
        self.language = os.environ["ASR_LANGUAGE"]  # Get language environment (获取语言环境)
        # Initialize ASR based on language and wake-up method (根据语言和唤醒方法初始化ASR)
        if not self.offline and self.asr_provider == 'stepfun':
            self.asr = None
        else:
            if self.awake_method == 'xf':
                if self.offline:
                    self.asr = speech.OfflineRealTimeASR(log=self.get_logger(), timeout=180)
                else:
                    if self.language == 'Chinese':
                        self.asr = speech.RealTimeASR(log=self.get_logger())
                    else:
                        self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                        self.asr.update_session(model=asr_model, language='en')
            else:
                if self.offline:
                    self.asr = speech.OfflineRealTimeASR(log=self.get_logger(), timeout=180)
                else:
                    if self.language == 'Chinese':
                        self.asr = speech.RealTimeASR(log=self.get_logger())
                    else:
                        self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                        self.asr.update_session(model=asr_model, language='en')
        
        # Create publishers and services (创建发布者和服务)
        self._configure_asr_audio()

        self.asr_pub = self.create_publisher(String, '~/asr_result', 1)  # ASR result publisher (语音识别结果发布者)
        self.wakeup_pub = self.create_publisher(Bool, '~/wakeup', 1)  # Wake-up status publisher (唤醒状态发布者)
        self.awake_angle_pub = self.create_publisher(Int32, '~/angle', 1)  # Wake-up angle publisher (唤醒角度发布者)
        self.create_service(SetInt32, '~/set_mode', self.set_mode_srv)  # Set mode service (设置模式服务)
        self.create_service(SetBool, '~/enable_wakeup', self.enable_wakeup_srv)  # Enable wake-up service (启用唤醒服务)

        # Start publisher callback thread (启动发布回调线程)
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)  # Initialization complete service (初始化完成服务)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def _set_asr_attrs(self, names, value):
        changed = []
        for name in names:
            if hasattr(self.asr, name):
                try:
                    setattr(self.asr, name, value)
                    changed.append(name)
                except Exception:
                    pass
        if not changed and names:
            try:
                setattr(self.asr, names[0], value)
                changed.append(names[0])
            except Exception:
                pass
        return changed

    def _configure_asr_audio(self):
        if self.offline or not hasattr(self, 'asr') or self.asr is None:
            return

        changed = []
        if self.asr_channels > 0:
            changed += self._set_asr_attrs(
                [
                    'channels', 'channel', 'channel_count', 'channelCount',
                    'CHANNELS', 'CHANNEL', 'CHANNEL_COUNT',
                    'audio_channels', 'input_channels',
                ],
                self.asr_channels,
            )
        if self.asr_input_device_index >= 0:
            changed += self._set_asr_attrs(
                ['input_device_index', 'device_index', 'device', 'DEVICE'],
                self.asr_input_device_index,
            )

        if changed:
            self.get_logger().info(
                'ASR audio override: channels=%d input_device_index=%d attrs=%s'
                % (self.asr_channels, self.asr_input_device_index, ','.join(changed))
            )

    def _has_online_asr_key(self):
        if self.offline or self.language != 'Chinese':
            return True
        if self.asr_provider == 'stepfun':
            return bool(self.stepfun_api_key)
        return bool(api_key or getattr(dashscope, 'api_key', None))

    def _reset_asr(self):
        if self.asr_provider == 'stepfun' or not hasattr(self, 'asr') or self.asr is None:
            return

        old_asr = self.asr
        for method_name in ('close', 'stop', 'shutdown', 'release', 'terminate'):
            method = getattr(old_asr, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
        try:
            del old_asr
        except Exception:
            pass
        gc.collect()
        time.sleep(0.2)

        if self.offline:
            self.asr = speech.OfflineRealTimeASR(log=self.get_logger(), timeout=180)
        elif self.language == 'Chinese':
            self.asr = speech.RealTimeASR(log=self.get_logger())
        else:
            self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
            self.asr.update_session(model=asr_model, language='en')
        self._configure_asr_audio()

    def _stepfun_asr(self):
        pcm_path = None
        try:
            with tempfile.NamedTemporaryFile(prefix='stepfun_asr_', suffix='.pcm', delete=False) as fp:
                pcm_path = fp.name

            duration = int(math.ceil(self.stepfun_record_seconds))
            cmd = [
                'arecord',
                '-q',
                '-D', self.stepfun_asr_device,
                '-f', 'S16_LE',
                '-r', '16000',
                '-c', '1',
                '-t', 'raw',
                '-d', str(duration),
                pcm_path,
            ]
            self.get_logger().info(
                'Recording StepFun ASR audio: device=%s seconds=%d'
                % (self.stepfun_asr_device, duration)
            )
            subprocess.run(cmd, check=True, timeout=duration + 3)

            with open(pcm_path, 'rb') as fp:
                audio_data = fp.read()
            if not audio_data:
                return ''

            payload = {
                'audio': {
                    'data': base64.b64encode(audio_data).decode('ascii'),
                    'input': {
                        'transcription': {
                            'language': 'zh' if self.language == 'Chinese' else 'en',
                            'model': self.stepfun_asr_model,
                            'enable_itn': True,
                        },
                        'format': {
                            'type': 'pcm',
                            'codec': 'pcm_s16le',
                            'rate': 16000,
                            'bits': 16,
                            'channel': 1,
                        },
                    },
                },
            }
            request = urllib.request.Request(
                self.stepfun_asr_endpoint,
                data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
                headers={
                    'Content-Type': 'application/json',
                    'Accept': 'text/event-stream',
                    'Authorization': 'Bearer ' + self.stepfun_api_key,
                },
                method='POST',
            )
            with urllib.request.urlopen(request, timeout=30) as response:
                return self._parse_stepfun_sse(response)
        finally:
            if pcm_path and os.path.exists(pcm_path):
                try:
                    os.unlink(pcm_path)
                except Exception:
                    pass

    def _parse_stepfun_sse(self, response):
        final_text = ''
        delta_text = ''
        last_text = ''
        for raw_line in response:
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line or not line.startswith('data:'):
                continue
            data = line[5:].strip()
            if not data or data == '[DONE]':
                continue
            try:
                event = json.loads(data)
            except Exception:
                continue
            event_type = event.get('type', '')
            event_text = self._stepfun_event_text(event)
            if event_type == 'transcript.text.done' or event_type.endswith('.done') or 'final' in event_type:
                final_text = event_text or final_text
            elif event_type == 'transcript.text.delta':
                delta_text += event.get('delta', '') or event_text
            elif event_type == 'error':
                raise RuntimeError(event.get('message', 'StepFun ASR error'))
            elif event_text:
                last_text = event_text
        return (final_text or delta_text or last_text).strip()

    def _stepfun_event_text(self, event):
        if not isinstance(event, dict):
            return ''
        for key in ('text', 'delta', 'transcript', 'result'):
            value = event.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip()
        for key in ('output', 'data', 'audio', 'transcription'):
            value = event.get(key)
            if isinstance(value, dict):
                text = self._stepfun_event_text(value)
                if text:
                    return text
        return ''

    def get_node_state(self, request, response):
        """

        Node state service callback(获取节点状态服务回调)
        """
        return response

    def record(self, mode, angle=None):
        """

        Record and recognize speech(录音并识别语音)
        
        Args:
            mode (int):  Working mode(工作模式)
            angle (int, optional): Wake-up angle (唤醒角度)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'asr...')
        if not self._has_online_asr_key():
            if self.asr_provider == 'stepfun':
                self.get_logger().error(
                    'StepFun ASR API key is empty. Set STEPFUN_API_KEY or STEP_API_KEY before launch.'
                )
            else:
                self.get_logger().error(
                    'Online Chinese ASR API key is empty. Set ALIYUN_API_KEY or DASHSCOPE_API_KEY before launch.'
                )
            self.enable_wakeup = True
            return

        try:
            if self.offline:
                asr_result = self.asr.asr()
                # asr_result = self.asr.punctuation(asr_result, self.punct_model, self.sherpa_onnx)
                if self.language != 'Chinese':
                    asr_result = asr_result.lower()
            else:
                if self.asr_provider == 'stepfun':
                    asr_result = self._stepfun_asr()
                elif self.language == 'Chinese':
                    asr_result = self.asr.asr(model=asr_model)  # Start recording and recognition (开启录音并识别)
                else:
                    asr_result = self.asr.asr()
        except Exception as e:
            self.get_logger().error(f'ASR failed: {e}')
            self.enable_wakeup = True
            self._reset_asr()
            try:
                speech.play_audio(error_audio_path)
            except Exception:
                pass
            return
        if asr_result: 
            speech.play_audio(dong_audio_path)  # Play prompt sound (播放提示音)
            if self.awake_method == 'xf' and self.mode == 1: 
                msg = Int32()
                msg.data = int(angle)
                self.awake_angle_pub.publish(msg)  # Publish wake-up angle (发布唤醒角度)
            asr_msg = String()
            asr_msg.data = asr_result
            self.asr_pub.publish(asr_msg)  # Publish ASR result (发布语音识别结果)
            self.enable_wakeup = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'publish asr result:' + asr_result)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'no voice detect')
            speech.play_audio(dong_audio_path)
            if mode == 1:
                speech.play_audio(no_voice_audio_path)  # Play no voice prompt (播放无语音提示)

    def pub_callback(self):
        """

        Publisher callback thread function, handling wake-up and recording(发布回调线程函数，处理唤醒和录音)
        """
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        while self.running:
            if self.enable_wakeup:
                if self.mode == 1:  # Wake-up mode (唤醒模式)
                    result = self.kws.wakeup()
                    if result:
                        self.wakeup_pub.publish(Bool(data=True))  # Publish wake-up status (发布唤醒状态)
                        speech.play_audio(wakeup_audio_path)  # Play wake-up prompt (唤醒播放提示音)
                        self.record(self.mode, result)  # Record and recognize (录音并识别)
                    else:
                        time.sleep(0.02)
                elif self.mode == 2:  # Direct recording mode (直接录音模式)
                    self.record(self.mode)
                    self.mode = 1
                elif self.mode == 3:  #  Another recording mode(另一种录音模式)
                    self.record(self.mode)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)
        if rclpy.ok():
            rclpy.shutdown()

    def enable_wakeup_srv(self, request, response):
        """

        Enable wake-up service callback(启用唤醒服务回调)
        
        Args:
            request: Request object (请求对象)
            response: Response object (响应对象)
            
        Returns:
            response: Service response (服务响应)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_wakeup'))
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        self.enable_wakeup = request.data  # Set wake-up status (设置唤醒状态)
        response.success = True
        return response 

    def set_mode_srv(self, request, response):
        """

        Set mode service callback(设置模式服务回调)
        
        Args:
            request: Request object (请求对象)
            response:  Response object(响应对象)
            
        Returns:
            response: Service response(服务响应)
        """
        self.get_logger().info(f'\033[1;32mset mode: {request.data}\033[0m')
        self.kws.start()  # Start wake-up device (启动唤醒设备)
        self.mode = int(request.data)  # Set working mode (设置工作模式)
        if self.mode != 1:
            self.enable_wakeup = True  # Enable wake-up in non-wake-up mode (非唤醒模式下启用唤醒)
        response.success = True
        return response 

def main():
    """

    Main function(主函数)
    """
    node = VocalDetect('vocal_detect')  # Create voice detection node (创建语音检测节点)
    try:
        rclpy.spin(node)  # Run node (运行节点)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        if rclpy.ok():
            rclpy.shutdown()  # Shutdown ROS2 (关闭ROS2)

if __name__ == "__main__":
    main()
