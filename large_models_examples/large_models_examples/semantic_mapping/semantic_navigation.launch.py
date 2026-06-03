import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _model_classes(model_name):
    garbage_names = [
        'BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks',
        'Ketchup', 'Marker', 'OralLiquidBottle', 'Plate', 'PlasticBottle',
        'StorageBattery', 'Toothbrush', 'Umbrella'
    ]
    traffic_names = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
    yolo_names = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
        'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
        'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
        'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
        'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
        'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
        'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
        'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush'
    ]
    if 'garbage' in model_name:
        return garbage_names, 'obb'
    if 'traffic' in model_name:
        return traffic_names, 'detect'
    return yolo_names, 'detect'


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _select_slam_map_path(map_name):
    source_slam_path = '/home/ubuntu/ros2_ws/src/slam'
    source_map_yaml = os.path.join(source_slam_path, 'maps', map_name + '.yaml')
    if os.path.exists(source_map_yaml):
        os.environ['need_compile'] = 'False'
        return source_slam_path, source_map_yaml

    install_slam_path = '/home/ubuntu/ros2_ws/install/slam/share/slam'
    install_map_yaml = os.path.join(install_slam_path, 'maps', map_name + '.yaml')
    if os.path.exists(install_map_yaml):
        os.makedirs(os.path.dirname(source_map_yaml), exist_ok=True)
        for suffix in ('.yaml', '.pgm'):
            src = os.path.join(install_slam_path, 'maps', map_name + suffix)
            dst = os.path.join(source_slam_path, 'maps', map_name + suffix)
            if os.path.exists(src):
                shutil.copy2(src, dst)
        os.environ['need_compile'] = 'False'
        return source_slam_path, source_map_yaml

    return source_slam_path, source_map_yaml


def launch_setup(context):
    os.environ.setdefault('need_compile', 'False')

    map_default = 'semantic_map'
    robot_name_default = os.environ.get('HOST', 'robot')
    master_name_default = os.environ.get('MASTER', 'master')
    model_name_default = 'yolo26n'
    semantic_map_file_default = '~/.ros/semantic_voxel_map.json'
    conf_default = '0.60'
    mode_default = '1'
    camera_topic_default = '/depth_cam/rgb0/image_raw'
    awake_words_default = 'wang2 ji4'
    asr_provider_default = 'stepfun'
    asr_channels_default = '1'
    asr_input_device_index_default = '-1'
    stepfun_asr_model_default = 'stepaudio-2.5-asr'
    stepfun_asr_endpoint_default = 'https://api.stepfun.com/v1/audio/asr/sse'
    stepfun_asr_device_default = 'default'
    stepfun_record_seconds_default = '4.0'
    llm_provider_default = 'stepfun'
    llm_model_default = os.environ.get('STEPFUN_LLM_MODEL', 'step-3.5-flash')
    llm_base_url_default = os.environ.get('STEPFUN_BASE_URL', 'https://api.stepfun.com/v1')
    llm_api_key_default = ''
    tts_provider_default = 'stepfun'
    stepfun_tts_model_default = os.environ.get('STEPFUN_TTS_MODEL', 'stepaudio-2.5-tts')
    stepfun_tts_endpoint_default = os.environ.get('STEPFUN_TTS_ENDPOINT', 'https://api.stepfun.com/v1/audio/speech')
    stepfun_tts_voice_default = os.environ.get('STEPFUN_TTS_VOICE', 'cixingnansheng')
    stepfun_tts_format_default = os.environ.get('STEPFUN_TTS_FORMAT', 'wav')
    use_rviz_default = 'true'
    stand_off_distance_default = '0.30'
    min_observations_default = '2'
    live_objects_timeout_default = '8.0'
    update_semantic_map_default = 'false'
    depth_topic_default = '/depth_cam/depth0/image_raw'
    camera_info_topic_default = '/depth_cam/rgb0/camera_info'
    objects_topic_default = '/yolo/object_detect'
    max_marker_voxels_default = '400'
    enable_path_scoring_default = 'true'
    max_path_score_candidates_default = '4'
    path_scoring_timeout_default = '0.35'
    path_scoring_total_timeout_default = '1.0'
    approach_angle_samples_default = '12'
    max_goal_attempts_default = '4'
    vehicle_width_default = '0.25'
    preferred_obstacle_margin_default = '0.08'
    max_clearance_check_default = '0.45'
    goal_yaw_mode_default = 'path_heading'
    retry_on_stall_default = 'true'
    stall_timeout_default = '8.0'
    stall_min_movement_default = '0.06'
    accept_near_goal_distance_default = '0.18'
    origin_x_default = '0.0'
    origin_y_default = '0.0'
    origin_yaw_default = '0.0'

    map_name = LaunchConfiguration('map', default=map_default).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=robot_name_default)
    master_name = LaunchConfiguration('master_name', default=master_name_default)
    model_name = LaunchConfiguration('model_name', default=model_name_default).perform(context)
    semantic_map_file_value = LaunchConfiguration('map_file', default=semantic_map_file_default).perform(context)
    semantic_map_file = LaunchConfiguration('map_file', default=semantic_map_file_default)
    conf = LaunchConfiguration('conf', default=conf_default)
    mode = LaunchConfiguration('mode', default=mode_default)
    camera_topic = LaunchConfiguration('camera_topic', default=camera_topic_default)
    awake_words = LaunchConfiguration('chinese_awake_words', default=awake_words_default)
    asr_provider = LaunchConfiguration('asr_provider', default=asr_provider_default)
    asr_channels = LaunchConfiguration('asr_channels', default=asr_channels_default)
    asr_input_device_index = LaunchConfiguration('asr_input_device_index', default=asr_input_device_index_default)
    stepfun_asr_model = LaunchConfiguration('stepfun_asr_model', default=stepfun_asr_model_default)
    stepfun_asr_endpoint = LaunchConfiguration('stepfun_asr_endpoint', default=stepfun_asr_endpoint_default)
    stepfun_asr_device = LaunchConfiguration('stepfun_asr_device', default=stepfun_asr_device_default)
    stepfun_record_seconds = LaunchConfiguration('stepfun_record_seconds', default=stepfun_record_seconds_default)
    llm_provider = LaunchConfiguration('llm_provider', default=llm_provider_default)
    llm_model = LaunchConfiguration('llm_model', default=llm_model_default)
    llm_base_url = LaunchConfiguration('llm_base_url', default=llm_base_url_default)
    llm_api_key = LaunchConfiguration('llm_api_key', default=llm_api_key_default)
    tts_provider = LaunchConfiguration('tts_provider', default=tts_provider_default)
    stepfun_tts_model = LaunchConfiguration('stepfun_tts_model', default=stepfun_tts_model_default)
    stepfun_tts_endpoint = LaunchConfiguration('stepfun_tts_endpoint', default=stepfun_tts_endpoint_default)
    stepfun_tts_voice = LaunchConfiguration('stepfun_tts_voice', default=stepfun_tts_voice_default)
    stepfun_tts_format = LaunchConfiguration('stepfun_tts_format', default=stepfun_tts_format_default)
    use_rviz = LaunchConfiguration('use_rviz', default=use_rviz_default)
    stand_off_distance = LaunchConfiguration('stand_off_distance', default=stand_off_distance_default)
    min_observations = LaunchConfiguration('min_observations', default=min_observations_default)
    live_objects_timeout = LaunchConfiguration('live_objects_timeout', default=live_objects_timeout_default)
    depth_topic = LaunchConfiguration('depth_topic', default=depth_topic_default)
    camera_info_topic = LaunchConfiguration('camera_info_topic', default=camera_info_topic_default)
    objects_topic = LaunchConfiguration('objects_topic', default=objects_topic_default)
    max_marker_voxels = LaunchConfiguration('max_marker_voxels', default=max_marker_voxels_default)
    enable_path_scoring = LaunchConfiguration('enable_path_scoring', default=enable_path_scoring_default)
    max_path_score_candidates = LaunchConfiguration(
        'max_path_score_candidates',
        default=max_path_score_candidates_default,
    )
    path_scoring_timeout = LaunchConfiguration('path_scoring_timeout', default=path_scoring_timeout_default)
    path_scoring_total_timeout = LaunchConfiguration(
        'path_scoring_total_timeout',
        default=path_scoring_total_timeout_default,
    )
    approach_angle_samples = LaunchConfiguration('approach_angle_samples', default=approach_angle_samples_default)
    max_goal_attempts = LaunchConfiguration('max_goal_attempts', default=max_goal_attempts_default)
    vehicle_width = LaunchConfiguration('vehicle_width', default=vehicle_width_default)
    preferred_obstacle_margin = LaunchConfiguration(
        'preferred_obstacle_margin',
        default=preferred_obstacle_margin_default,
    )
    max_clearance_check = LaunchConfiguration('max_clearance_check', default=max_clearance_check_default)
    goal_yaw_mode = LaunchConfiguration('goal_yaw_mode', default=goal_yaw_mode_default)
    retry_on_stall = LaunchConfiguration('retry_on_stall', default=retry_on_stall_default)
    stall_timeout = LaunchConfiguration('stall_timeout', default=stall_timeout_default)
    stall_min_movement = LaunchConfiguration('stall_min_movement', default=stall_min_movement_default)
    accept_near_goal_distance = LaunchConfiguration(
        'accept_near_goal_distance',
        default=accept_near_goal_distance_default,
    )
    origin_x = LaunchConfiguration('origin_x', default=origin_x_default)
    origin_y = LaunchConfiguration('origin_y', default=origin_y_default)
    origin_yaw = LaunchConfiguration('origin_yaw', default=origin_yaw_default)
    update_semantic_map_value = LaunchConfiguration(
        'update_semantic_map',
        default=update_semantic_map_default,
    ).perform(context)
    update_semantic_map_enabled = _as_bool(update_semantic_map_value)

    classes, task = _model_classes(model_name)
    slam_package_path, map_yaml = _select_slam_map_path(map_name)

    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        navigation_package_path = get_package_share_directory('navigation')
    else:
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

    large_models_package_path = get_package_share_directory('large_models')
    semantic_mapping_share = get_package_share_directory('semantic_mapping')
    semantic_mapping_path = os.path.join(semantic_mapping_share, 'rviz')
    semantic_map_path = os.path.abspath(os.path.expanduser(os.path.expandvars(semantic_map_file_value)))
    semantic_nav2_params = os.path.join(semantic_mapping_share, 'config', 'nav2_params_semantic.yaml')
    semantic_controller_params = os.path.join(semantic_mapping_share, 'config', 'nav2_controller_teb_semantic.yaml')

    launch_arguments = [
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('robot_name', default_value=robot_name_default),
        DeclareLaunchArgument('master_name', default_value=master_name_default),
        DeclareLaunchArgument('model_name', default_value=model_name_default),
        DeclareLaunchArgument('map_file', default_value=semantic_map_file_default),
        DeclareLaunchArgument('conf', default_value=conf_default),
        DeclareLaunchArgument('mode', default_value=mode_default),
        DeclareLaunchArgument('camera_topic', default_value=camera_topic_default),
        DeclareLaunchArgument('chinese_awake_words', default_value=awake_words_default),
        DeclareLaunchArgument('asr_provider', default_value=asr_provider_default),
        DeclareLaunchArgument('asr_channels', default_value=asr_channels_default),
        DeclareLaunchArgument('asr_input_device_index', default_value=asr_input_device_index_default),
        DeclareLaunchArgument('stepfun_asr_model', default_value=stepfun_asr_model_default),
        DeclareLaunchArgument('stepfun_asr_endpoint', default_value=stepfun_asr_endpoint_default),
        DeclareLaunchArgument('stepfun_asr_device', default_value=stepfun_asr_device_default),
        DeclareLaunchArgument('stepfun_record_seconds', default_value=stepfun_record_seconds_default),
        DeclareLaunchArgument('llm_provider', default_value=llm_provider_default),
        DeclareLaunchArgument('llm_model', default_value=llm_model_default),
        DeclareLaunchArgument('llm_base_url', default_value=llm_base_url_default),
        DeclareLaunchArgument('llm_api_key', default_value=llm_api_key_default),
        DeclareLaunchArgument('tts_provider', default_value=tts_provider_default),
        DeclareLaunchArgument('stepfun_tts_model', default_value=stepfun_tts_model_default),
        DeclareLaunchArgument('stepfun_tts_endpoint', default_value=stepfun_tts_endpoint_default),
        DeclareLaunchArgument('stepfun_tts_voice', default_value=stepfun_tts_voice_default),
        DeclareLaunchArgument('stepfun_tts_format', default_value=stepfun_tts_format_default),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz_default),
        DeclareLaunchArgument('stand_off_distance', default_value=stand_off_distance_default),
        DeclareLaunchArgument('min_observations', default_value=min_observations_default),
        DeclareLaunchArgument('live_objects_timeout', default_value=live_objects_timeout_default),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic_default),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic_default),
        DeclareLaunchArgument('objects_topic', default_value=objects_topic_default),
        DeclareLaunchArgument('max_marker_voxels', default_value=max_marker_voxels_default),
        DeclareLaunchArgument('update_semantic_map', default_value=update_semantic_map_default),
        DeclareLaunchArgument('enable_path_scoring', default_value=enable_path_scoring_default),
        DeclareLaunchArgument('max_path_score_candidates', default_value=max_path_score_candidates_default),
        DeclareLaunchArgument('path_scoring_timeout', default_value=path_scoring_timeout_default),
        DeclareLaunchArgument('path_scoring_total_timeout', default_value=path_scoring_total_timeout_default),
        DeclareLaunchArgument('approach_angle_samples', default_value=approach_angle_samples_default),
        DeclareLaunchArgument('max_goal_attempts', default_value=max_goal_attempts_default),
        DeclareLaunchArgument('vehicle_width', default_value=vehicle_width_default),
        DeclareLaunchArgument('preferred_obstacle_margin', default_value=preferred_obstacle_margin_default),
        DeclareLaunchArgument('max_clearance_check', default_value=max_clearance_check_default),
        DeclareLaunchArgument('goal_yaw_mode', default_value=goal_yaw_mode_default),
        DeclareLaunchArgument('retry_on_stall', default_value=retry_on_stall_default),
        DeclareLaunchArgument('stall_timeout', default_value=stall_timeout_default),
        DeclareLaunchArgument('stall_min_movement', default_value=stall_min_movement_default),
        DeclareLaunchArgument('accept_near_goal_distance', default_value=accept_near_goal_distance_default),
        DeclareLaunchArgument('origin_x', default_value=origin_x_default),
        DeclareLaunchArgument('origin_y', default_value=origin_y_default),
        DeclareLaunchArgument('origin_yaw', default_value=origin_yaw_default),
    ]

    if not os.path.exists(map_yaml):
        return launch_arguments + [
            LogInfo(msg=f'[semantic_navigation] ERROR: 2D navigation map not found: {map_yaml}'),
            LogInfo(msg='[semantic_navigation] Run semantic_map_builder first and wait for semantic_map.pgm/.yaml to be saved before navigation.'),
        ]

    semantic_map_logs = []
    if os.path.exists(semantic_map_path):
        semantic_map_logs.append(LogInfo(msg=f'[semantic_navigation] Loading semantic object map: {semantic_map_path}'))
    else:
        semantic_map_logs.append(LogInfo(msg=f'[semantic_navigation] WARNING: semantic object map not found yet: {semantic_map_path}'))

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/navigation.launch.py')),
        launch_arguments={
            'sim': 'false',
            'map': map_name,
            'robot_name': robot_name,
            'master_name': master_name,
            'use_teb': 'true',
            'params_file': semantic_nav2_params,
            'controller_param': semantic_controller_params,
        }.items(),
    )

    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(large_models_package_path, 'launch/vocal_detect.launch.py')),
            launch_arguments={
                'mode': mode,
                'chinese_awake_words': awake_words,
                'asr_provider': asr_provider,
                'asr_channels': asr_channels,
                'asr_input_device_index': asr_input_device_index,
                'stepfun_asr_model': stepfun_asr_model,
                'stepfun_asr_endpoint': stepfun_asr_endpoint,
                'stepfun_asr_device': stepfun_asr_device,
                'stepfun_record_seconds': stepfun_record_seconds,
            }.items(),
        )
    agent_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(large_models_package_path, 'launch/agent_process.launch.py')),
        launch_arguments={'camera_topic': camera_topic}.items(),
    )
    tts_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(large_models_package_path, 'launch/tts_node.launch.py')),
        launch_arguments={
            'tts_provider': tts_provider,
            'stepfun_tts_model': stepfun_tts_model,
            'stepfun_tts_endpoint': stepfun_tts_endpoint,
            'stepfun_tts_voice': stepfun_tts_voice,
            'stepfun_tts_format': stepfun_tts_format,
        }.items(),
    )

    yolo_node = Node(
        package='example',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'classes': classes,
            'engine': model_name,
            'conf': conf,
            'task': task,
            'display': False,
            'start': True,
        }],
    )

    live_mapper_node = Node(
        package='semantic_mapping',
        executable='semantic_voxel_mapper',
        output='screen',
        parameters=[{
            'map_file': semantic_map_file,
            'map_frame': 'map',
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'objects_topic': objects_topic,
            'load_existing_map': True,
            'publish_voxel_markers': True,
            'integrate_depth_map': False,
            'max_marker_voxels': max_marker_voxels,
        }],
    )

    saved_marker_node = Node(
        package='semantic_mapping',
        executable='semantic_map_marker_publisher',
        output='screen',
        parameters=[{
            'map_file': semantic_map_file,
            'map_frame': 'map',
            'publish_voxel_markers': True,
            'max_marker_voxels': max_marker_voxels,
            'publish_origin_marker': True,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'origin_yaw': origin_yaw,
        }],
    )

    navigation_controller_node = Node(
        package='large_models_examples',
        executable='navigation_controller',
        output='screen',
        parameters=[{'map_frame': 'map', 'nav_goal': '/nav_goal', 'goal_timeout': 180.0}],
    )

    semantic_navigation_tool_node = Node(
        package='semantic_mapping',
        executable='semantic_navigation_tool',
        output='screen',
        parameters=[{
            'map_file': semantic_map_file,
            'stand_off_distance': stand_off_distance,
            'min_observations': min_observations,
            'live_objects_timeout': live_objects_timeout,
            'map_topic': '/map',
            'goal_clearance': 0.28,
            'occupancy_threshold': 50,
            'allow_unknown_goals': False,
            'approach_angle_samples': approach_angle_samples,
            'max_goal_attempts': max_goal_attempts,
            'enable_path_scoring': enable_path_scoring,
            'max_path_score_candidates': max_path_score_candidates,
            'path_scoring_timeout': path_scoring_timeout,
            'path_scoring_total_timeout': path_scoring_total_timeout,
            'vehicle_width': vehicle_width,
            'preferred_obstacle_margin': preferred_obstacle_margin,
            'max_clearance_check': max_clearance_check,
            'goal_yaw_mode': goal_yaw_mode,
            'retry_on_stall': retry_on_stall,
            'stall_timeout': stall_timeout,
            'stall_min_movement': stall_min_movement,
            'accept_near_goal_distance': accept_near_goal_distance,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'origin_yaw': origin_yaw,
            'llm_provider': llm_provider,
            'llm_model': llm_model,
            'llm_base_url': llm_base_url,
            'llm_api_key': llm_api_key,
        }],
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            os.path.join(semantic_mapping_path, 'semantic_mapping.rviz'),
        ],
        output='screen',
    )

    semantic_mapper_actions = [
        TimerAction(period=8.0, actions=[saved_marker_node])
    ]
    semantic_map_log = (
        '[semantic_navigation] Semantic map is read-only: saved voxel markers are shown, '
        'but no new voxels are added. Set update_semantic_map:=true to update online.'
    )
    if update_semantic_map_enabled:
        semantic_mapper_actions = [
            TimerAction(period=5.0, actions=[yolo_node]),
            TimerAction(period=8.0, actions=[live_mapper_node]),
        ]
        semantic_map_log = '[semantic_navigation] Online semantic map update is enabled.'

    return [
        *launch_arguments,
        LogInfo(msg=f'[semantic_navigation] Loading 2D navigation map: {map_yaml}'),
        *semantic_map_logs,
        LogInfo(msg='[semantic_navigation] Starting Nav2 base; camera, LiDAR, robot model and map_server are owned by navigation.launch.py.'),
        LogInfo(msg=semantic_map_log),
        LogInfo(msg=['[semantic_navigation] Voice wake word pinyin: ', awake_words]),
        navigation_launch,
        vocal_detect_launch,
        agent_process_launch,
        tts_node_launch,
        TimerAction(period=22.0, actions=[rviz_node], condition=IfCondition(use_rviz)),
        *semantic_mapper_actions,
        TimerAction(
            period=32.0,
            actions=[
                navigation_controller_node,
                semantic_navigation_tool_node,
            ],
        ),
        TimerAction(period=50.0, actions=[
            LogInfo(msg='[semantic_navigation] Ready. Say the target object name after Nav2 reports ready. Semantic targets are loaded from the saved JSON unless update_semantic_map is true.')
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
