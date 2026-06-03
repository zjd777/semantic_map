import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb0/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    offline = LaunchConfiguration('offline', default='false').perform(context)
    offline_arg = DeclareLaunchArgument('offline', default_value=offline)
    interruption = LaunchConfiguration('interruption', default=False)
    interruption_arg = DeclareLaunchArgument('interruption', default_value=interruption)
    avoidance_enabled = LaunchConfiguration('avoidance_enabled', default='true')
    avoidance_enabled_arg = DeclareLaunchArgument('avoidance_enabled', default_value=avoidance_enabled)
    avoidance_debug = LaunchConfiguration('avoidance_debug', default='false')
    avoidance_debug_arg = DeclareLaunchArgument('avoidance_debug', default_value=avoidance_debug)
    raw_cmd_vel_topic = LaunchConfiguration('raw_cmd_vel_topic', default='/vllm_track/cmd_vel_raw')
    raw_cmd_vel_topic_arg = DeclareLaunchArgument('raw_cmd_vel_topic', default_value=raw_cmd_vel_topic)
    output_cmd_vel_topic = LaunchConfiguration('output_cmd_vel_topic', default='/controller/cmd_vel')
    output_cmd_vel_topic_arg = DeclareLaunchArgument('output_cmd_vel_topic', default_value=output_cmd_vel_topic)
    target_state_topic = LaunchConfiguration('target_state_topic', default='/vllm_track/target_state')
    target_state_topic_arg = DeclareLaunchArgument('target_state_topic', default_value=target_state_topic)
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value=scan_topic)
    safety_distance = LaunchConfiguration('safety_distance', default='0.3')
    safety_distance_arg = DeclareLaunchArgument('safety_distance', default_value=safety_distance)
    target_stop_distance = LaunchConfiguration('target_stop_distance', default='20.0')
    target_stop_distance_arg = DeclareLaunchArgument('target_stop_distance', default_value=target_stop_distance)
    slow_distance = LaunchConfiguration('slow_distance', default='0.3')
    slow_distance_arg = DeclareLaunchArgument('slow_distance', default_value=slow_distance)
    scan_angle = LaunchConfiguration('scan_angle', default='120.0')
    scan_angle_arg = DeclareLaunchArgument('scan_angle', default_value=scan_angle)
    obstacle_angle = LaunchConfiguration('obstacle_angle', default='40.0')
    obstacle_angle_arg = DeclareLaunchArgument('obstacle_angle', default_value=obstacle_angle)
    blocked_avoidance_timeout = LaunchConfiguration('blocked_avoidance_timeout', default='3.0')
    blocked_avoidance_timeout_arg = DeclareLaunchArgument('blocked_avoidance_timeout', default_value=blocked_avoidance_timeout)
    target_ignore_margin = LaunchConfiguration('target_ignore_margin', default='0.12')
    target_ignore_margin_arg = DeclareLaunchArgument('target_ignore_margin', default_value=target_ignore_margin)
    target_ignore_angle = LaunchConfiguration('target_ignore_angle', default='25.0')
    target_ignore_angle_arg = DeclareLaunchArgument('target_ignore_angle', default_value=target_ignore_angle)
    danger_reverse_speed = LaunchConfiguration('danger_reverse_speed', default='0.03')
    danger_reverse_speed_arg = DeclareLaunchArgument('danger_reverse_speed', default_value=danger_reverse_speed)
    avoid_forward_speed = LaunchConfiguration('avoid_forward_speed', default='0.0')
    avoid_forward_speed_arg = DeclareLaunchArgument('avoid_forward_speed', default_value=avoid_forward_speed)
    avoid_turn_speed = LaunchConfiguration('avoid_turn_speed', default='0.12')
    avoid_turn_speed_arg = DeclareLaunchArgument('avoid_turn_speed', default_value=avoid_turn_speed)
    avoid_strafe_speed = LaunchConfiguration('avoid_strafe_speed', default='0.12')
    avoid_strafe_speed_arg = DeclareLaunchArgument('avoid_strafe_speed', default_value=avoid_strafe_speed)
    invert_avoid_direction = LaunchConfiguration('invert_avoid_direction', default='false')
    invert_avoid_direction_arg = DeclareLaunchArgument('invert_avoid_direction', default_value=invert_avoid_direction)

    slam_package_path = get_package_share_directory('slam')
    large_models_package_path = get_package_share_directory('large_models') 
    
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': 'false',
            'master_name': os.environ['MASTER'],
            'robot_name': os.environ['HOST']
        }.items(),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={
            'mode': mode,
            'offline': offline,
            'camera_topic': camera_topic,
        }.items(),
    )

    vllm_track_node = Node(
        package='large_models_examples',
        executable='vllm_track',
        output='screen',
        parameters=[{
            'interruption': interruption,
            'cmd_vel_topic': raw_cmd_vel_topic,
            'target_stop_distance': target_stop_distance,
            'target_state_topic': target_state_topic,
        }],
    )

    obstacle_avoidance_filter_node = Node(
        package='large_models_examples',
        executable='obstacle_avoidance_filter',
        output='screen',
        parameters=[{
            'enabled': avoidance_enabled,
            'input_cmd_vel_topic': raw_cmd_vel_topic,
            'output_cmd_vel_topic': output_cmd_vel_topic,
            'target_state_topic': target_state_topic,
            'scan_topic': scan_topic,
            'safety_distance': safety_distance,
            'slow_distance': slow_distance,
            'scan_angle': scan_angle,
            'obstacle_angle': obstacle_angle,
            'blocked_avoidance_timeout': blocked_avoidance_timeout,
            'target_ignore_margin': target_ignore_margin,
            'target_ignore_angle': target_ignore_angle,
            'danger_reverse_speed': danger_reverse_speed,
            'avoid_forward_speed': avoid_forward_speed,
            'avoid_turn_speed': avoid_turn_speed,
            'avoid_strafe_speed': avoid_strafe_speed,
            'invert_avoid_direction': invert_avoid_direction,
            'debug': avoidance_debug,
        }],
    )

    # rqt
    calibrate_rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='calibrate_rqt_reconfigure'
    )

    return [mode_arg,
            offline_arg,
            interruption_arg,
            camera_topic_arg,
            avoidance_enabled_arg,
            avoidance_debug_arg,
            raw_cmd_vel_topic_arg,
            output_cmd_vel_topic_arg,
            target_state_topic_arg,
            scan_topic_arg,
            safety_distance_arg,
            target_stop_distance_arg,
            slow_distance_arg,
            scan_angle_arg,
            obstacle_angle_arg,
            blocked_avoidance_timeout_arg,
            target_ignore_margin_arg,
            target_ignore_angle_arg,
            danger_reverse_speed_arg,
            avoid_forward_speed_arg,
            avoid_turn_speed_arg,
            avoid_strafe_speed_arg,
            invert_avoid_direction_arg,
            base_launch,
            large_models_launch,
            obstacle_avoidance_filter_node,
            vllm_track_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
