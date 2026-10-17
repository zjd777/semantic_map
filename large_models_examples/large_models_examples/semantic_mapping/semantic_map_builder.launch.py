import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def launch_setup(context):
    os.environ.setdefault('need_compile', 'False')
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        slam_package_path = get_package_share_directory('slam')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'

    semantic_mapping_path = os.path.dirname(os.path.realpath(__file__))
    default_map_save_prefix = '~/ros2_ws/src/slam/maps/semantic_map'

    model_name = LaunchConfiguration('model_name', default='yolo26n').perform(context)
    classes, task = _model_classes(model_name)
    robot_name_value = LaunchConfiguration('robot_name', default='/').perform(context)

    map_file = LaunchConfiguration('map_file', default='~/.ros/semantic_voxel_map.json')
    conf = LaunchConfiguration('conf', default='0.60')
    display = LaunchConfiguration('display', default='false')
    use_controller = LaunchConfiguration('use_controller', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_lidar_slam = LaunchConfiguration('use_lidar_slam', default='true')
    use_map_saver = LaunchConfiguration('use_map_saver', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    robot_name = LaunchConfiguration('robot_name', default=robot_name_value)
    map_frame = LaunchConfiguration('map_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_save_prefix = LaunchConfiguration('map_save_prefix', default=default_map_save_prefix)
    semantic_overlay_enabled = LaunchConfiguration('semantic_overlay_enabled', default='false')
    semantic_obstacle_radius = LaunchConfiguration('semantic_obstacle_radius', default='0.18')
    semantic_min_observations_for_occupancy = LaunchConfiguration(
        'semantic_min_observations_for_occupancy',
        default='2',
    )
    semantic_min_confidence_for_occupancy = LaunchConfiguration(
        'semantic_min_confidence_for_occupancy',
        default='0.60',
    )
    semantic_occupancy_classes = LaunchConfiguration('semantic_occupancy_classes', default='all')
    semantic_publish_period = LaunchConfiguration('semantic_publish_period', default='2.0')
    voxel_size = LaunchConfiguration('voxel_size', default='0.12')
    sample_stride = LaunchConfiguration('sample_stride', default='8')
    object_depth_percentile = LaunchConfiguration('object_depth_percentile', default='35.0')
    object_depth_band = LaunchConfiguration('object_depth_band', default='0.35')
    load_existing_map = LaunchConfiguration('load_existing_map', default='false')
    publish_voxel_markers = LaunchConfiguration('publish_voxel_markers', default='true')
    integrate_depth_map = LaunchConfiguration('integrate_depth_map', default='true')
    depth_map_stride = LaunchConfiguration('depth_map_stride', default='36')
    depth_map_period = LaunchConfiguration('depth_map_period', default='2.0')
    max_depth_points_per_update = LaunchConfiguration('max_depth_points_per_update', default='700')
    max_occupied_voxels = LaunchConfiguration('max_occupied_voxels', default='6000')
    max_marker_voxels = LaunchConfiguration('max_marker_voxels', default='700')
    max_saved_voxels = LaunchConfiguration('max_saved_voxels', default='12000')
    min_occupied_count_for_marker = LaunchConfiguration('min_occupied_count_for_marker', default='2')
    min_occupied_count_for_save = LaunchConfiguration('min_occupied_count_for_save', default='2')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth_cam/depth0/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/depth_cam/rgb0/camera_info')
    objects_topic = LaunchConfiguration('objects_topic', default='/yolo/object_detect')
    camera_frame = LaunchConfiguration('camera_frame', default='')

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')
        ),
        launch_arguments={
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')
        ),
        launch_arguments={
            'lidar_frame': lidar_frame,
            'scan_topic': scan_topic,
            'scan_raw': scan_raw,
        }.items(),
    )

    lidar_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/slam_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'scan_topic': scan_topic,
            'enable_save': 'true',
        }.items(),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/teleop_key_control.launch.py')
        ),
        launch_arguments={'robot_name': robot_name}.items(),
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            os.path.join(semantic_mapping_path, 'semantic_mapping.rviz'),
        ],
        output='screen',
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
            'display': ParameterValue(display, value_type=bool),
            'start': True,
        }],
    )

    mapper_node = Node(
        package='large_models_examples',
        executable='semantic_voxel_mapper',
        output='screen',
        parameters=[{
            'map_file': map_file,
            'map_frame': map_frame,
            'camera_frame': camera_frame,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'objects_topic': objects_topic,
            'load_existing_map': ParameterValue(load_existing_map, value_type=bool),
            'publish_period': semantic_publish_period,
            'voxel_size': voxel_size,
            'sample_stride': sample_stride,
            'object_depth_percentile': object_depth_percentile,
            'object_depth_band': object_depth_band,
            'publish_voxel_markers': ParameterValue(publish_voxel_markers, value_type=bool),
            'integrate_depth_map': ParameterValue(integrate_depth_map, value_type=bool),
            'depth_map_stride': depth_map_stride,
            'depth_map_period': depth_map_period,
            'max_depth_points_per_update': max_depth_points_per_update,
            'max_occupied_voxels': max_occupied_voxels,
            'max_marker_voxels': max_marker_voxels,
            'max_saved_voxels': max_saved_voxels,
            'min_occupied_count_for_marker': min_occupied_count_for_marker,
            'min_occupied_count_for_save': min_occupied_count_for_save,
        }],
    )

    occupancy_grid_saver_node = Node(
        package='large_models_examples',
        executable='semantic_occupancy_grid_saver',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'output_prefix': map_save_prefix,
            'semantic_overlay_enabled': ParameterValue(semantic_overlay_enabled, value_type=bool),
            'semantic_objects_topic': '/semantic_map/objects',
            'semantic_obstacle_radius': semantic_obstacle_radius,
            'semantic_min_observations_for_occupancy': semantic_min_observations_for_occupancy,
            'semantic_min_confidence_for_occupancy': semantic_min_confidence_for_occupancy,
            'semantic_occupancy_classes': semantic_occupancy_classes,
        }],
    )

    return [
        DeclareLaunchArgument('model_name', default_value=model_name),
        DeclareLaunchArgument('conf', default_value=conf),
        DeclareLaunchArgument('display', default_value=display),
        DeclareLaunchArgument('use_controller', default_value=use_controller),
        DeclareLaunchArgument('use_lidar', default_value=use_lidar),
        DeclareLaunchArgument('use_lidar_slam', default_value=use_lidar_slam),
        DeclareLaunchArgument('use_map_saver', default_value=use_map_saver),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),
        DeclareLaunchArgument('use_teleop', default_value=use_teleop),
        DeclareLaunchArgument('robot_name', default_value=robot_name),
        DeclareLaunchArgument('map_frame', default_value=map_frame),
        DeclareLaunchArgument('odom_frame', default_value=odom_frame),
        DeclareLaunchArgument('base_frame', default_value=base_frame),
        DeclareLaunchArgument('lidar_frame', default_value=lidar_frame),
        DeclareLaunchArgument('scan_topic', default_value=scan_topic),
        DeclareLaunchArgument('scan_raw', default_value=scan_raw),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('map_save_prefix', default_value=map_save_prefix),
        DeclareLaunchArgument('semantic_overlay_enabled', default_value=semantic_overlay_enabled),
        DeclareLaunchArgument('semantic_obstacle_radius', default_value=semantic_obstacle_radius),
        DeclareLaunchArgument(
            'semantic_min_observations_for_occupancy',
            default_value=semantic_min_observations_for_occupancy,
        ),
        DeclareLaunchArgument(
            'semantic_min_confidence_for_occupancy',
            default_value=semantic_min_confidence_for_occupancy,
        ),
        DeclareLaunchArgument('semantic_occupancy_classes', default_value=semantic_occupancy_classes),
        DeclareLaunchArgument('semantic_publish_period', default_value=semantic_publish_period),
        DeclareLaunchArgument('voxel_size', default_value=voxel_size),
        DeclareLaunchArgument('sample_stride', default_value=sample_stride),
        DeclareLaunchArgument('object_depth_percentile', default_value=object_depth_percentile),
        DeclareLaunchArgument('object_depth_band', default_value=object_depth_band),
        DeclareLaunchArgument('map_file', default_value=map_file),
        DeclareLaunchArgument('load_existing_map', default_value=load_existing_map),
        DeclareLaunchArgument('publish_voxel_markers', default_value=publish_voxel_markers),
        DeclareLaunchArgument('integrate_depth_map', default_value=integrate_depth_map),
        DeclareLaunchArgument('depth_map_stride', default_value=depth_map_stride),
        DeclareLaunchArgument('depth_map_period', default_value=depth_map_period),
        DeclareLaunchArgument('max_depth_points_per_update', default_value=max_depth_points_per_update),
        DeclareLaunchArgument('max_occupied_voxels', default_value=max_occupied_voxels),
        DeclareLaunchArgument('max_marker_voxels', default_value=max_marker_voxels),
        DeclareLaunchArgument('max_saved_voxels', default_value=max_saved_voxels),
        DeclareLaunchArgument('min_occupied_count_for_marker', default_value=min_occupied_count_for_marker),
        DeclareLaunchArgument('min_occupied_count_for_save', default_value=min_occupied_count_for_save),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic),
        DeclareLaunchArgument('objects_topic', default_value=objects_topic),
        DeclareLaunchArgument('camera_frame', default_value=camera_frame),
        LogInfo(msg='[semantic_map_builder] Fresh live mapping: LiDAR builds /map, RGB-D+YOLO builds semantic objects, no old semantic map, no Nav2.'),
        LogInfo(msg=['[semantic_map_builder] Semantic mapper topics: depth=', depth_topic, ', camera_info=', camera_info_topic, ', objects=', objects_topic]),
        depth_camera_launch,
        TimerAction(period=1.0, actions=[controller_launch], condition=IfCondition(use_controller)),
        TimerAction(period=2.0, actions=[lidar_launch], condition=IfCondition(use_lidar)),
        TimerAction(period=5.0, actions=[lidar_slam_launch], condition=IfCondition(use_lidar_slam)),
        TimerAction(period=7.0, actions=[occupancy_grid_saver_node], condition=IfCondition(use_map_saver)),
        yolo_node,
        mapper_node,
        TimerAction(period=2.0, actions=[rviz_node], condition=IfCondition(use_rviz)),
        TimerAction(period=2.0, actions=[teleop_launch], condition=IfCondition(use_teleop)),
        TimerAction(period=4.0, actions=[
            LogInfo(msg='[semantic_map_builder] Ready. Keyboard drives /controller/cmd_vel; RViz shows /map, /scan, RobotModel and semantic object voxels in map.')
        ]),
        TimerAction(period=8.0, actions=[
            LogInfo(msg='[semantic_map_builder] Map files are saved only by manual service calls: /semantic_occupancy_grid_saver/save and /semantic_voxel_mapper/save.')
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
