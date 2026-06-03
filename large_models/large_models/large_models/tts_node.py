#!/usr/bin/env python3
# encoding: utf-8
import os
import threading
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

from large_models.config import *
from speech import speech


class TTSNode(Node):
    """Text-to-speech node."""

    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.declare_parameter('offline', 'false')
        self.declare_parameter('tts_provider', os.getenv('TTS_PROVIDER', 'config'))
        self.declare_parameter('stepfun_tts_model', os.getenv('STEPFUN_TTS_MODEL', stepfun_tts_model))
        self.declare_parameter('stepfun_tts_endpoint', os.getenv('STEPFUN_TTS_ENDPOINT', stepfun_tts_endpoint))
        self.declare_parameter('stepfun_tts_voice', os.getenv('STEPFUN_TTS_VOICE', stepfun_tts_voice))
        self.declare_parameter('stepfun_tts_format', os.getenv('STEPFUN_TTS_FORMAT', stepfun_tts_format))

        self.text = None
        speech.set_volume(80)

        self.offline = os.environ["ASR_MODE"] == 'offline'
        self.language = os.environ["ASR_LANGUAGE"]
        self.tts_provider = str(self.get_parameter('tts_provider').value or 'config').strip().lower()
        self.stepfun_api_key = (
            os.getenv('STEPFUN_API_KEY')
            or os.getenv('STEP_API_KEY')
            or os.getenv('STEPFUN_KEY')
            or stepfun_api_key
        )
        self.stepfun_tts_model = str(self.get_parameter('stepfun_tts_model').value or '').strip()
        self.stepfun_tts_endpoint = str(self.get_parameter('stepfun_tts_endpoint').value or '').strip()
        self.stepfun_tts_voice = str(self.get_parameter('stepfun_tts_voice').value or '').strip()
        self.stepfun_tts_format = str(self.get_parameter('stepfun_tts_format').value or 'wav').strip().lower()

        if self.offline:
            import sherpa_onnx
            self.sherpa_onnx = sherpa_onnx
            model_path = f'{sherpa_onnx_path}/{offline_tts}'
            if self.language == 'Chinese':
                self.tts = speech.OfflineRealTimeTTS(
                    provider="cuda",
                    debug=1,
                    matcha_acoustic_model=os.path.join(model_path, 'model-steps-3.onnx'),
                    matcha_vocoder=os.path.join(sherpa_onnx_path, 'vocos-22khz-univ.onnx'),
                    matcha_lexicon=os.path.join(model_path, 'lexicon.txt'),
                    matcha_tokens=os.path.join(model_path, 'tokens.txt'),
                    tts_rule_fsts=f'{model_path}/phone.fst,{model_path}/date.fst,{model_path}/number.fst',
                    log=self.get_logger(),
                    sherpa=self.sherpa_onnx,
                )
            else:
                self.tts = speech.OfflineRealTimeTTS(
                    provider="cuda",
                    debug=1,
                    vits_model=os.path.join(model_path, f'{offline_tts}.onnx'),
                    vits_lexicon=os.path.join(model_path, 'lexicon.txt'),
                    vits_tokens=os.path.join(model_path, 'tokens.txt'),
                    log=self.get_logger(),
                    sherpa=self.sherpa_onnx,
                )
        elif self.tts_provider == 'stepfun':
            self.tts = None
            if not self.stepfun_api_key:
                self.get_logger().warn('STEPFUN_API_KEY/STEP_API_KEY is empty; StepFun TTS will fail')
            self.get_logger().info(
                f'StepFun TTS enabled: model={self.stepfun_tts_model}, voice={self.stepfun_tts_voice}'
            )
        else:
            if self.language == 'Chinese':
                self.tts = speech.RealTimeTTS(log=self.get_logger())
            else:
                self.tts = speech.RealTimeOpenAITTS(log=self.get_logger())

        self.play_finish_pub = self.create_publisher(Bool, '~/play_finish', 1)
        self.create_subscription(String, '~/tts_text', self.tts_callback, 1)

        threading.Thread(target=self.tts_process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        return response

    def tts_callback(self, msg):
        self.get_logger().info(f'tts text: {msg.data}')
        self.text = msg.data

    def tts_process(self):
        while rclpy.ok():
            if self.text is None:
                time.sleep(0.01)
                continue

            text = self.text
            self.text = None
            try:
                if text == '':
                    speech.play_audio(no_voice_audio_path)
                elif self.offline:
                    self.tts.tts(text, sid=offline_tts_speaker)
                elif self.tts_provider == 'stepfun':
                    self._stepfun_tts(text)
                else:
                    self.tts.tts(text, model=tts_model, voice=voice_model)
            except Exception as e:
                self.get_logger().error(f'TTS failed: {e}')

            msg = Bool()
            msg.data = True
            self.play_finish_pub.publish(msg)

    def _stepfun_tts(self, text):
        if not self.stepfun_api_key:
            raise RuntimeError('StepFun TTS API key is empty')
        if not self.stepfun_tts_endpoint:
            raise RuntimeError('StepFun TTS endpoint is empty')

        self.get_logger().info('StepFun speech synthesizer is opened.')
        response = requests.post(
            self.stepfun_tts_endpoint,
            headers={
                'Authorization': 'Bearer ' + self.stepfun_api_key,
                'Content-Type': 'application/json',
                'Accept': 'audio/' + self.stepfun_tts_format,
            },
            json={
                'model': self.stepfun_tts_model,
                'input': text,
                'voice': self.stepfun_tts_voice,
                'response_format': self.stepfun_tts_format,
            },
            timeout=30,
        )
        if response.status_code >= 400:
            raise RuntimeError(
                'StepFun TTS HTTP %s: %s' % (response.status_code, response.text[:300])
            )
        content_type = response.headers.get('content-type', '').lower()
        if 'json' in content_type:
            raise RuntimeError('StepFun TTS returned JSON: %s' % response.text[:300])

        with open(tts_audio_path, 'wb') as fp:
            fp.write(response.content)
        speech.play_audio(tts_audio_path)
        self.get_logger().info('StepFun speech synthesizer is completed.')


def main():
    node = TTSNode('tts_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
