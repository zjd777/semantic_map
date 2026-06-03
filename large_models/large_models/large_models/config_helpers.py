#!/usr/bin/env python3
# encoding: utf-8

import os

from large_models import config as cfg


def _value(name, default=''):
    return getattr(cfg, name, default) or default


def _normalize_provider(provider):
    return str(provider or 'config').strip().lower().replace('-', '_')


def _model_for(provider, model_type):
    is_vllm = str(model_type or '').lower() in ('vllm', 'vision', 'vision_llm')
    if provider in ('stepfun', 'step', 'step_fun'):
        if is_vllm:
            return _value('stepfun_vllm_model') or 'step-1o-vision-32k'
        return _value('stepfun_llm_model') or 'step-3.5-flash'
    if provider in ('aliyun', 'dashscope', 'qwen'):
        return _value('aliyun_vllm_model' if is_vllm else 'aliyun_llm_model')
    if provider in ('openrouter', 'router'):
        return _value('vllm_model')
    if provider in ('openai', 'gpt'):
        if is_vllm:
            return _value('openai_vllm_model') or _value('vllm_model')
        return _value('llm_model')
    return _value('vllm_model' if is_vllm else 'llm_model')


def get_llm_config(
    provider=None,
    model_type='llm',
    model='',
    api_key_override='',
    base_url_override='',
):
    provider = _normalize_provider(provider if provider is not None else os.getenv('LLM_PROVIDER', 'config'))
    if not api_key_override:
        api_key_override = os.getenv('LLM_API_KEY', '')
    if not base_url_override:
        base_url_override = os.getenv('LLM_BASE_URL', '')
    if not model:
        env_model_name = 'VLLM_MODEL' if str(model_type or '').lower() in ('vllm', 'vision', 'vision_llm') else 'LLM_MODEL'
        model = os.getenv(env_model_name, '')
    if provider in ('config', 'default', ''):
        return {
            'provider': 'config',
            'api_key': api_key_override or _value('api_key'),
            'base_url': base_url_override or _value('base_url'),
            'model': model or _model_for('config', model_type),
        }
    if provider in ('stepfun', 'step', 'step_fun'):
        return {
            'provider': 'stepfun',
            'api_key': api_key_override or _value('stepfun_api_key'),
            'base_url': base_url_override or _value('stepfun_base_url', 'https://api.stepfun.com/v1'),
            'model': model or _model_for(provider, model_type),
        }
    if provider in ('aliyun', 'dashscope', 'qwen'):
        return {
            'provider': 'aliyun',
            'api_key': api_key_override or _value('aliyun_api_key'),
            'base_url': base_url_override or _value('aliyun_base_url'),
            'model': model or _model_for(provider, model_type),
        }
    if provider in ('openrouter', 'router'):
        return {
            'provider': 'openrouter',
            'api_key': api_key_override or _value('vllm_api_key'),
            'base_url': base_url_override or _value('vllm_base_url'),
            'model': model or _model_for(provider, model_type),
        }
    if provider in ('openai', 'gpt'):
        return {
            'provider': 'openai',
            'api_key': api_key_override or _value('llm_api_key'),
            'base_url': base_url_override or _value('llm_base_url'),
            'model': model or _model_for(provider, model_type),
        }
    return get_llm_config(
        provider='config',
        model_type=model_type,
        model=model,
        api_key_override=api_key_override,
        base_url_override=base_url_override,
    )


def get_vllm_config(provider=None, model='', api_key_override='', base_url_override=''):
    if provider is None:
        provider = os.getenv('VLLM_PROVIDER', os.getenv('LLM_PROVIDER', 'config'))
    if not model:
        model = os.getenv('VLLM_MODEL', '')
    if not api_key_override:
        api_key_override = os.getenv('VLLM_API_KEY', '')
    if not base_url_override:
        base_url_override = os.getenv('VLLM_BASE_URL', '')
    return get_llm_config(
        provider=provider,
        model_type='vllm',
        model=model,
        api_key_override=api_key_override,
        base_url_override=base_url_override,
    )


def configure_llm_request(
    msg,
    model_type='llm',
    provider=None,
    model='',
    api_key_override='',
    base_url_override='',
    enable_search=False,
    enable_think=False,
):
    llm_config = get_llm_config(
        provider=provider,
        model_type=model_type,
        model=model,
        api_key_override=api_key_override,
        base_url_override=base_url_override,
    )
    msg.model_type = model_type
    msg.model = llm_config['model']
    msg.api_key = llm_config['api_key']
    msg.base_url = llm_config['base_url']
    if hasattr(msg, 'enable_search'):
        msg.enable_search = bool(enable_search)
    if hasattr(msg, 'enable_think'):
        msg.enable_think = bool(enable_think)
    return msg
