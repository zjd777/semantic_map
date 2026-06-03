import os

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    launch_list =  [
            ]
    tts_provider = LaunchConfiguration('tts_provider')
    stepfun_tts_model = LaunchConfiguration('stepfun_tts_model')
    stepfun_tts_endpoint = LaunchConfiguration('stepfun_tts_endpoint')
    stepfun_tts_voice = LaunchConfiguration('stepfun_tts_voice')
    stepfun_tts_format = LaunchConfiguration('stepfun_tts_format')

    tts_node = Node(
        package='large_models',
        executable='tts_node',
        output='screen',
        parameters=[{
            'tts_provider': tts_provider,
            'stepfun_tts_model': stepfun_tts_model,
            'stepfun_tts_endpoint': stepfun_tts_endpoint,
            'stepfun_tts_voice': stepfun_tts_voice,
            'stepfun_tts_format': stepfun_tts_format,
        }],
    )
    launch_list.append(tts_node)
    return launch_list

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tts_provider', default_value=os.environ.get('TTS_PROVIDER', 'config')),
        DeclareLaunchArgument('stepfun_tts_model', default_value=os.environ.get('STEPFUN_TTS_MODEL', 'stepaudio-2.5-tts')),
        DeclareLaunchArgument('stepfun_tts_endpoint', default_value=os.environ.get('STEPFUN_TTS_ENDPOINT', 'https://api.stepfun.com/v1/audio/speech')),
        DeclareLaunchArgument('stepfun_tts_voice', default_value=os.environ.get('STEPFUN_TTS_VOICE', 'cixingnansheng')),
        DeclareLaunchArgument('stepfun_tts_format', default_value=os.environ.get('STEPFUN_TTS_FORMAT', 'wav')),
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
