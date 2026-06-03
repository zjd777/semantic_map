#!/usr/bin/env python3
# encoding: utf-8
import os
import dashscope
from pathlib import Path

###Offline###
offline_llm = 'qwen3:1.7b'
if os.environ["ASR_LANGUAGE"] == 'Chinese':
    offline_asr = 'sherpa-onnx-streaming-zipformer-zh-xlarge-int8-2025-06-30'
    offline_tts = 'matcha-icefall-zh-baker'
else:
    offline_asr = 'sherpa-onnx-streaming-zipformer-en-2023-06-21'
    offline_tts = 'vits-ljs'
offline_tts_speaker = 100
offline_punct_model = 'sherpa-onnx-punct-ct-transformer-zh-en-vocab272727-2024-04-12'
ollama_host = 'http://localhost:11434'
sherpa_onnx_path = os.path.join(Path.home(), 'third_party/sherpa-onnx')
###

###Only in China###
# 阶跃星辰key
stepfun_api_key = os.getenv('STEPFUN_API_KEY') or os.getenv('STEP_API_KEY') or '3pNPVHGPiIyTZfqjnB1HK7IRTUstNBAY1YdbKmOZiVhRbtXIGZUwT6Vt6fdhPgGkI'
stepfun_base_url = os.getenv('STEPFUN_BASE_URL', 'https://api.stepfun.com/v1')
stepfun_llm_model = os.getenv('STEPFUN_LLM_MODEL', 'step-3.5-flash')
stepfun_tts_endpoint = os.getenv('STEPFUN_TTS_ENDPOINT', 'https://api.stepfun.com/v1/audio/speech')
stepfun_tts_model = os.getenv('STEPFUN_TTS_MODEL', 'stepaudio-2.5-tts')
stepfun_tts_voice = os.getenv('STEPFUN_TTS_VOICE', 'cixingnansheng')
stepfun_tts_format = os.getenv('STEPFUN_TTS_FORMAT', 'wav')
#'step-1v-8k'/'step-1o-vision-32k'/'step-1.5v-mini'
stepfun_vllm_model = 'step-1o-vision-32k'

# 阿里云key
aliyun_api_key = os.getenv('ALIYUN_API_KEY') or os.getenv('DASHSCOPE_API_KEY', '')
aliyun_base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
aliyun_llm_model = 'qwen-max-latest'#'qwen-turbo'#'qwen-max-latest'
aliyun_vllm_model = 'qwen-vl-max-latest'
aliyun_tts_model = 'sambert-zhinan-v1'
aliyun_asr_model = 'paraformer-realtime-v2'
aliyun_voice_model = ''
######

###Internationally###
vllm_api_key = os.getenv('OPENROUTER_API_KEY', '')
vllm_base_url = 'https://openrouter.ai/api/v1'
vllm_model = 'qwen/qwen2.5-vl-72b-instruct'

llm_api_key = os.getenv('OPENAI_API_KEY', '')
llm_base_url = 'https://api.openai.com/v1'
llm_model = 'gpt-4o-mini'
openai_llm_model = llm_model
openai_vllm_model = 'gpt-4o'
openai_tts_model = 'tts-1'
openai_asr_model = 'whisper-1'
openai_voice_model = 'onyx'
######

openrouter_tools_model = os.getenv('OPENROUTER_TOOLS_MODEL', 'qwen/qwen3-max')


def _normalize_provider(provider):
    return str(provider or '').strip().lower().replace('-', '_')


def _provider_from_env(kind, fallback):
    env_name = f'{kind.upper()}_PROVIDER'
    return _normalize_provider(os.getenv(env_name) or os.getenv('MODEL_PROVIDER') or fallback)


def get_llm_config(provider=None, model_type='llm', model=None, api_key_override='', base_url_override=''):
    language = os.environ.get("ASR_LANGUAGE", "Chinese")
    if provider is None:
        if language == 'Chinese':
            fallback = 'stepfun'
        elif model_type == 'llm_tools':
            fallback = 'openrouter'
        else:
            fallback = 'openai'
        provider = _provider_from_env('llm', fallback)
    provider = _normalize_provider(provider)

    if provider in ('stepfun', 'step', 'step_fun'):
        return {
            'provider': 'stepfun',
            'api_key': api_key_override or os.getenv('STEPFUN_API_KEY') or os.getenv('STEP_API_KEY') or stepfun_api_key,
            'base_url': base_url_override or os.getenv('STEPFUN_BASE_URL') or stepfun_base_url,
            'model': model or os.getenv('STEPFUN_LLM_MODEL') or stepfun_llm_model,
        }
    if provider in ('aliyun', 'dashscope', 'qwen'):
        return {
            'provider': 'aliyun',
            'api_key': api_key_override or aliyun_api_key,
            'base_url': base_url_override or aliyun_base_url,
            'model': model or os.getenv('ALIYUN_LLM_MODEL') or aliyun_llm_model,
        }
    if provider in ('openrouter', 'router'):
        return {
            'provider': 'openrouter',
            'api_key': api_key_override or vllm_api_key,
            'base_url': base_url_override or vllm_base_url,
            'model': model or os.getenv('OPENROUTER_LLM_MODEL') or openrouter_tools_model,
        }
    if provider in ('openai', 'gpt'):
        return {
            'provider': 'openai',
            'api_key': api_key_override or llm_api_key,
            'base_url': base_url_override or llm_base_url,
            'model': model or os.getenv('OPENAI_LLM_MODEL') or openai_llm_model,
        }

    if language == 'Chinese':
        return get_llm_config(provider='stepfun', model_type=model_type, model=model)
    if model_type == 'llm_tools':
        return get_llm_config(provider='openrouter', model_type=model_type, model=model)
    return get_llm_config(provider='openai', model_type=model_type, model=model)


def configure_llm_request(request, model_type='llm', provider=None, model=None, enable_search=False, enable_think=False):
    config = get_llm_config(provider=provider, model_type=model_type, model=model)
    request.model_type = model_type
    request.model = config['model']
    request.api_key = config['api_key']
    request.base_url = config['base_url']
    if hasattr(request, 'enable_search'):
        request.enable_search = bool(enable_search)
    if hasattr(request, 'enable_think'):
        request.enable_think = bool(enable_think)
    return request


def get_vllm_config(provider=None, model=None, api_key_override='', base_url_override=''):
    language = os.environ.get("ASR_LANGUAGE", "Chinese")
    provider = _provider_from_env('vllm', 'stepfun' if language == 'Chinese' else 'openrouter') if provider is None else _normalize_provider(provider)

    if provider in ('stepfun', 'step', 'step_fun'):
        return {
            'provider': 'stepfun',
            'api_key': api_key_override or os.getenv('STEPFUN_API_KEY') or os.getenv('STEP_API_KEY') or stepfun_api_key,
            'base_url': base_url_override or os.getenv('STEPFUN_BASE_URL') or stepfun_base_url,
            'model': model or os.getenv('STEPFUN_VLLM_MODEL') or stepfun_vllm_model,
        }
    if provider in ('aliyun', 'dashscope', 'qwen'):
        return {
            'provider': 'aliyun',
            'api_key': api_key_override or aliyun_api_key,
            'base_url': base_url_override or aliyun_base_url,
            'model': model or os.getenv('ALIYUN_VLLM_MODEL') or aliyun_vllm_model,
        }
    if provider in ('openai', 'gpt'):
        return {
            'provider': 'openai',
            'api_key': api_key_override or llm_api_key,
            'base_url': base_url_override or llm_base_url,
            'model': model or os.getenv('OPENAI_VLLM_MODEL') or openai_vllm_model,
        }
    return {
        'provider': 'openrouter',
        'api_key': api_key_override or vllm_api_key,
        'base_url': base_url_override or vllm_base_url,
        'model': model or os.getenv('OPENROUTER_VLLM_MODEL') or vllm_model,
    }


def configure_vllm_request(request, model_type='vllm', provider=None, model=None):
    config = get_vllm_config(provider=provider, model=model)
    if hasattr(request, 'model_type'):
        request.model_type = model_type
    request.model = config['model']
    request.api_key = config['api_key']
    request.base_url = config['base_url']
    return request

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    # The actual key used for invocation(实际调用的key)
    api_key = aliyun_api_key
    dashscope.api_key = aliyun_api_key
    base_url = aliyun_base_url
    asr_model = aliyun_asr_model
    tts_model = aliyun_tts_model
    voice_model = aliyun_voice_model
    llm_model = aliyun_llm_model
    vllm_model = aliyun_vllm_model
else:
    api_key = llm_api_key
    os.environ["OPENAI_API_KEY"] = api_key
    base_url = llm_base_url
    asr_model = openai_asr_model
    tts_model = openai_tts_model
    voice_model = openai_voice_model

# Get the path of the current program(获取程序所在路径)
code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline')
    else:
        audio_path = os.path.join(code_path, 'resources/audio')
else:
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline/en')
    else:
        audio_path = os.path.join(code_path, 'resources/audio/en')

# Path to the recorded audio(录音音频的路径)
recording_audio_path = os.path.join(audio_path, 'recording.wav')

# Path to the synthesized (TTS) audio(语音合成音频的路径)
tts_audio_path = os.path.join(audio_path, "tts_audio.wav")

# Path to the startup audio(启动音频的路径)
start_audio_path = os.path.join(audio_path, "start_audio.wav")

# Path to the wake-up response audio(唤醒回答音频的路径)
wakeup_audio_path = os.path.join(audio_path, "wakeup.wav")

# Path to the error audio(出错音频的路径)
error_audio_path = os.path.join(audio_path, "error.wav")

# Path to the audio played when no sound is detected(没有检测到声音时音频的路径)
no_voice_audio_path = os.path.join(audio_path, "no_voice.wav")

# Path to the audio played when recording is complete(录音完成时音频的路径)
dong_audio_path = os.path.join(audio_path, "dong.wav")

record_finish_audio_path = os.path.join(audio_path, "record_finish.wav")

start_track_audio_path = os.path.join(audio_path, "start_track.wav")

track_fail_audio_path = os.path.join(audio_path, "track_fail.wav")
