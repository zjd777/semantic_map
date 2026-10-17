from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    subscribe_agent_result = LaunchConfiguration('subscribe_agent_result', default='false')
    agent_result_topic = LaunchConfiguration('agent_result_topic', default='agent_process/result')
    subscribe_agent_result_arg = DeclareLaunchArgument(
        'subscribe_agent_result',
        default_value=subscribe_agent_result,
    )
    agent_result_topic_arg = DeclareLaunchArgument(
        'agent_result_topic',
        default_value=agent_result_topic,
    )

    launch_list =  [
            subscribe_agent_result_arg,
            agent_result_topic_arg,
            ]

    tts_node = Node(
        package='large_models',
        executable='tts_node',
        output='screen',
        parameters=[{
            'subscribe_agent_result': subscribe_agent_result,
            'agent_result_topic': agent_result_topic,
        }],
    )
    launch_list.append(tts_node)
    return launch_list

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
