#!/usr/bin/env python3
# encoding: utf-8

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from interfaces.srv import SetPose2D
from large_models.config import api_key, base_url, llm_model, start_audio_path
from large_models_msgs.msg import Tools
from large_models_msgs.srv import SetModel, SetString, SetTools
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from speech import speech
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger


SEMANTIC_TOOLS = [
    {
        'type': 'function',
        'function': {
            'name': 'get_semantic_objects',
            'description': '查询语义地图里已经识别到的物体。用户问“语义地图里有什么”或“有哪些点”时必须调用。Use this to list objects known in the semantic voxel map.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'class_name': {
                        'type': 'string',
                        'description': '可选物体类别，例如 椅子/chair、瓶子/bottle、人/person、行李箱/suitcase、马桶/toilet、电视/tv。',
                    }
                },
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'semantic_object_navigation',
            'description': '导航到语义地图里识别到的物体，不需要预先给定导航点。Use this when the user says go to an object.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'object_name': {
                        'type': 'string',
                        'description': '目标物体名称，例如 椅子/chair、桌子/dining table、瓶子/bottle、人/person、行李箱/suitcase、马桶/toilet、电视/tv。',
                    },
                    'selection': {
                        'type': 'string',
                        'description': '选择哪个实例：nearest 最近，latest 最新，most_confident 最可信。',
                        'enum': ['nearest', 'latest', 'most_confident'],
                    },
                },
                'required': ['object_name'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'return_to_origin',
            'description': '返回地图原点，也就是小车建图和导航开始时的出发点。用户说“返回原点”“回到起点”“回出发点”时必须调用。',
            'parameters': {
                'type': 'object',
                'properties': {},
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'track_semantic_object',
            'description': '开始追踪语义地图中的目标物体。Use this when the user asks to follow or track an object.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'object_name': {
                        'type': 'string',
                        'description': '要追踪的物体类别，例如 人/person、瓶子/bottle、椅子/chair。',
                    }
                },
                'required': ['object_name'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'stop_tracking',
            'description': '停止当前语义目标追踪或取消正在执行的语义导航。',
            'parameters': {
                'type': 'object',
                'properties': {},
            },
        },
    },
]


LANGUAGE = os.environ.get('ASR_LANGUAGE', 'Chinese')

if LANGUAGE == 'Chinese':
    PROMPT = """
你是一个可以使用语义体素地图的机器人导航助手。
语义地图由 RGB-D 相机、YOLO 识别结果和 TF 自动构建，里面保存了物体类别和地图坐标。

用户说“去椅子那里”“带我去瓶子旁边”“导航到人附近”时，必须调用 semantic_object_navigation。
用户说“找到椅子”“找一下行李箱”“寻找某个东西”时，也表示要导航过去，必须调用 semantic_object_navigation。
用户问“地图里有什么”“语义地图里有什么”“告诉我语义地图里有什么”“有哪些语义点”“哪里有椅子”“你认识哪些物体”时，必须调用 get_semantic_objects。
用户说“返回原点”“回到起点”“回出发点”“回到最开始的位置”时，必须调用 return_to_origin，不要把原点当作物体查找。
用户说“跟着人”“追踪瓶子”时，调用 track_semantic_object。
用户说“停止追踪”“停下当前追踪”时，调用 stop_tracking。

物体名称可以是中文或英文。常见中文目标：人、椅子、桌子、瓶子、水瓶、杯子、书、背包、行李箱、箱子、沙发、马桶、厕所、电视。
当前语义地图只支持物体类别导航，不支持颜色、衣服、材质等属性筛选。如果用户说“蓝色行李箱”“穿白衣服的人”，请提取核心类别为“行李箱”或“人”，不要把颜色和衣服当成类别。
如果用户说的是“杯子/水瓶/瓶子”“行李箱/箱子/旅行箱”这类口语名称，请选择最接近的语义类别。
地图里的物体可能已经设置自定义名称，例如“门口行李箱”。用户按自定义名称查找或导航时，保留该名称调用工具。
工具调用完成后，用简短自然的中文回复用户。
"""
else:
    PROMPT = """
You are a robot assistant that can use a semantic voxel map.
The map is built online from RGB-D camera, YOLO detections, and TF.

When the user asks where objects are, call get_semantic_objects.
When the user asks the robot to go to an object, call semantic_object_navigation.
When the user asks the robot to return home, to its origin, or to its start point, call return_to_origin.
When the user asks the robot to follow or track an object, call track_semantic_object.
Object names may be Chinese or English. Prefer object classes that exist in the map.
Reply briefly and naturally after each tool result.
"""


CLASS_ALIASES = {
    '\u4eba': 'person',
    '\u884c\u4eba': 'person',
    '\u6905\u5b50': 'chair',
    '\u51f3\u5b50': 'chair',
    '\u684c\u5b50': 'dining table',
    '\u9910\u684c': 'dining table',
    '\u6c99\u53d1': 'couch',
    '\u74f6\u5b50': 'bottle',
    '\u6c34\u74f6': 'bottle',
    '\u676f\u5b50': 'cup',
    '\u624b\u673a': 'cell phone',
    '\u4e66': 'book',
    '\u80cc\u5305': 'backpack',
    '\u4e66\u5305': 'backpack',
    '\u884c\u674e\u7bb1': 'suitcase',
    '\u65c5\u884c\u7bb1': 'suitcase',
    '\u62c9\u6746\u7bb1': 'suitcase',
    '\u624b\u63d0\u7bb1': 'suitcase',
    '\u7bb1\u5b50': 'suitcase',
    '\u76d2\u5b50': 'suitcase',
    '\u4ea4\u901a\u706f': 'traffic light',
    '\u7ea2\u7eff\u706f': 'traffic light',
    '\u9a6c\u6876': 'toilet',
    '\u5395\u6240': 'toilet',
    '\u5750\u4fbf\u5668': 'toilet',
    '\u7535\u89c6': 'tv',
    '\u7535\u89c6\u673a': 'tv',
    '\u663e\u793a\u5668': 'tv',
    '\u6d88\u9632\u6813': 'fire hydrant',
}

CLASS_NAMES_ZH = {
    'person': '\u4eba',
    'chair': '\u6905\u5b50',
    'dining table': '\u684c\u5b50',
    'couch': '\u6c99\u53d1',
    'bottle': '\u74f6\u5b50',
    'cup': '\u676f\u5b50',
    'cell phone': '\u624b\u673a',
    'book': '\u4e66',
    'backpack': '\u80cc\u5305',
    'suitcase': '\u884c\u674e\u7bb1',
    'traffic light': '\u7ea2\u7eff\u706f',
    'toilet': '\u9a6c\u6876',
    'tv': '\u7535\u89c6',
    'fire hydrant': '\u6d88\u9632\u6813',
}


ATTRIBUTE_WORDS = [
    '\u7ea2\u8272', '\u84dd\u8272', '\u7eff\u8272', '\u9ec4\u8272', '\u767d\u8272', '\u9ed1\u8272',
    '\u7ea2', '\u84dd', '\u7eff', '\u9ec4', '\u767d', '\u9ed1',
    '\u7a7f\u767d\u8863\u670d\u7684', '\u7a7f\u9ed1\u8863\u670d\u7684', '\u7a7f\u7ea2\u8863\u670d\u7684',
    '\u7a7f\u84dd\u8863\u670d\u7684', '\u7a7f\u8863\u670d\u7684', '\u90a3\u4e2a', '\u6211\u7684',
]

ORIGIN_NAMES = {
    '\u539f\u70b9', '\u8d77\u70b9', '\u51fa\u53d1\u70b9', '\u521d\u59cb\u70b9',
    '\u521d\u59cb\u4f4d\u7f6e', '\u6700\u5f00\u59cb\u7684\u4f4d\u7f6e',
    'origin', 'home', 'start', 'start point',
}


def _expand_path(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path)))


def _normalize(text):
    return str(text or '').strip().lower().replace('_', ' ')


def _class_name_zh(class_name):
    normalized = _normalize(class_name)
    return CLASS_NAMES_ZH.get(normalized, normalized or '\u76ee\u6807')


class SemanticNavigationTool(Node):
    """LLM tool node for navigating to objects from the semantic voxel map."""

    def __init__(self):
        super().__init__('semantic_navigation_tool')

        self.declare_parameter('map_file', '~/.ros/semantic_voxel_map.json')
        self.declare_parameter('stand_off_distance', 0.30)
        self.declare_parameter('navigation_timeout', 180.0)
        self.declare_parameter('tracking_period', 2.0)
        self.declare_parameter('goal_update_distance', 0.35)
        self.declare_parameter('lost_timeout', 10.0)
        self.declare_parameter('live_objects_timeout', 8.0)
        self.declare_parameter('min_observations', 2)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_clearance', 0.28)
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('allow_unknown_goals', False)
        self.declare_parameter('approach_angle_samples', 16)
        self.declare_parameter('max_goal_attempts', 6)
        self.declare_parameter('enable_path_scoring', True)
        self.declare_parameter('planner_action_name', '/compute_path_to_pose')
        self.declare_parameter('max_path_score_candidates', 10)
        self.declare_parameter('path_scoring_timeout', 0.7)
        self.declare_parameter('vehicle_width', 0.25)
        self.declare_parameter('preferred_obstacle_margin', 0.08)
        self.declare_parameter('max_clearance_check', 0.45)
        self.declare_parameter('path_treat_unknown_as_obstacle', False)
        self.declare_parameter('goal_yaw_mode', 'path_heading')
        self.declare_parameter('retry_on_stall', True)
        self.declare_parameter('stall_timeout', 8.0)
        self.declare_parameter('stall_min_movement', 0.06)
        self.declare_parameter('accept_near_goal_distance', 0.18)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)

        self.map_file = _expand_path(self.get_parameter('map_file').value)
        self.stand_off_distance = float(self.get_parameter('stand_off_distance').value)
        self.navigation_timeout = float(self.get_parameter('navigation_timeout').value)
        self.tracking_period = float(self.get_parameter('tracking_period').value)
        self.goal_update_distance = float(self.get_parameter('goal_update_distance').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.live_objects_timeout = float(self.get_parameter('live_objects_timeout').value)
        self.min_observations = max(1, int(self.get_parameter('min_observations').value))
        self.map_topic = self.get_parameter('map_topic').value
        self.goal_clearance = max(0.0, float(self.get_parameter('goal_clearance').value))
        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.allow_unknown_goals = self._as_bool(self.get_parameter('allow_unknown_goals').value)
        self.approach_angle_samples = max(8, int(self.get_parameter('approach_angle_samples').value))
        self.max_goal_attempts = max(1, int(self.get_parameter('max_goal_attempts').value))
        self.enable_path_scoring = self._as_bool(self.get_parameter('enable_path_scoring').value)
        self.planner_action_name = self.get_parameter('planner_action_name').value
        self.max_path_score_candidates = max(1, int(self.get_parameter('max_path_score_candidates').value))
        self.path_scoring_timeout = max(0.1, float(self.get_parameter('path_scoring_timeout').value))
        self.vehicle_width = max(0.01, float(self.get_parameter('vehicle_width').value))
        self.preferred_obstacle_margin = max(0.0, float(self.get_parameter('preferred_obstacle_margin').value))
        self.max_clearance_check = max(0.05, float(self.get_parameter('max_clearance_check').value))
        self.path_treat_unknown_as_obstacle = self._as_bool(
            self.get_parameter('path_treat_unknown_as_obstacle').value
        )
        self.goal_yaw_mode = str(self.get_parameter('goal_yaw_mode').value).strip().lower()
        self.retry_on_stall = self._as_bool(self.get_parameter('retry_on_stall').value)
        self.stall_timeout = max(2.0, float(self.get_parameter('stall_timeout').value))
        self.stall_min_movement = max(0.01, float(self.get_parameter('stall_min_movement').value))
        self.accept_near_goal_distance = max(0.05, float(self.get_parameter('accept_near_goal_distance').value))
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_yaw = float(self.get_parameter('origin_yaw').value)
        self.goal_retry_distances = self._goal_retry_distances()

        self.cb_group = ReentrantCallbackGroup()
        self.tools = []
        self.current_pose = None
        self.reach_goal = False
        self.navigation_done = False
        self.tracking_target = None
        self.tracking_thread = None
        self.last_tracking_goal = None
        self.interrupt = False
        self.task_running = False
        self.live_objects = []
        self.live_objects_updated_at = 0.0
        self.live_objects_frame = ''
        self.nav_map = None
        self.nav_map_updated_at = 0.0

        self.set_tool_client = self.create_client(SetTools, 'agent_process/set_tool')
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_pose_client = self.create_client(SetPose2D, 'navigation_controller/set_pose')
        self.cancel_nav_client = self.create_client(Trigger, 'navigation_controller/cancel')
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup', callback_group=self.cb_group)
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name,
            callback_group=self.cb_group,
        )

        self.create_subscription(Tools, 'agent_process/tools', self.tools_callback, 1, callback_group=self.cb_group)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1)
        self.create_subscription(String, '/semantic_map/objects', self.semantic_objects_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=self.cb_group)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=self.cb_group)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, qos_profile)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos_profile)

        self.tools_result_pub = self.create_publisher(Tools, 'agent_process/tools_result', 1)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)

        self.create_timer(0.0, self.init_process, callback_group=self.cb_group)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def init_process(self):
        if hasattr(self, 'started') and self.started:
            return
        self.started = True

        self._wait_for_services()

        msg = SetModel.Request()
        msg.model_type = 'llm_tools'
        msg.model = 'qwen3-max' if LANGUAGE == 'Chinese' else llm_model
        msg.api_key = api_key
        msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        prompt = SetString.Request()
        prompt.data = PROMPT
        self.send_request(self.set_prompt_client, prompt)

        tools_msg = SetTools.Request()
        tools_msg.tools = [json.dumps(tool, ensure_ascii=False) for tool in SEMANTIC_TOOLS]
        self.send_request(self.set_tool_client, tools_msg)

        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        if os.path.exists(start_audio_path):
            speech.play_audio(start_audio_path)
        else:
            self.get_logger().warn(f'Start audio not found: {start_audio_path}')
        self._enable_wakeup(True)
        self.get_logger().info(f'Semantic navigation tool started, using {self.map_file}')

    def _wait_for_services(self):
        services = [
            (self.set_model_client, 'agent_process/set_model'),
            (self.set_prompt_client, 'agent_process/set_prompt'),
            (self.set_tool_client, 'agent_process/set_tool'),
            (self.set_pose_client, 'navigation_controller/set_pose'),
            (self.cancel_nav_client, 'navigation_controller/cancel'),
            (self.awake_client, 'vocal_detect/enable_wakeup'),
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=8.0):
                self.get_logger().warn(f'Service {name} not available yet')

    def get_node_state(self, request, response):
        return response

    def send_request(self, client, msg, timeout_sec=10.0):
        service_name = getattr(client, 'srv_name', 'service')
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'Service {service_name} not available')
            return None

        future = client.call_async(msg)
        start_time = time.time()
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                    return None
            if timeout_sec is not None and time.time() - start_time > timeout_sec:
                self.get_logger().warn(f'Service call timed out: {service_name}')
                return None
            time.sleep(0.01)
        return None

    def tools_callback(self, msg):
        self.get_logger().info(f'LLM tool call: {msg.name} {msg.data}')
        self.tools = [msg.id, msg.name, json.loads(msg.data or '{}')]

    def llm_result_callback(self, msg):
        if msg.data:
            self._speak(msg.data)

    def play_audio_finish_callback(self, msg):
        if msg.data:
            self._enable_wakeup(True)

    def wakeup_callback(self, msg):
        if not msg.data:
            return
        if self.task_running or self.tracking_target:
            self.get_logger().info('Wakeup received during semantic navigation, canceling current goal')
            self.interrupt = True
            self.tracking_target = None
            self._cancel_navigation()
        else:
            self.get_logger().info('Wakeup received, ready for a semantic navigation command')

    def amcl_pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z),
        )
        self.current_pose = {'x': pos.x, 'y': pos.y, 'yaw': yaw}

    def map_callback(self, msg):
        self.nav_map = msg
        self.nav_map_updated_at = time.time()

    def reach_goal_callback(self, msg):
        self.reach_goal = bool(msg.data)
        self.navigation_done = True

    def semantic_objects_callback(self, msg):
        try:
            data = json.loads(msg.data or '{}')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse live semantic objects: {e}', throttle_duration_sec=2.0)
            return

        objects = data.get('objects', [])
        if not isinstance(objects, list):
            return

        self.live_objects = objects
        self.live_objects_frame = data.get('frame_id', '')
        self.live_objects_updated_at = time.time()

    def process(self):
        while rclpy.ok():
            if not self.tools:
                time.sleep(0.02)
                continue

            tool_id, tool_name, args = self.tools
            self.tools = []
            self.interrupt = False
            self.task_running = tool_name in (
                'semantic_object_navigation', 'return_to_origin', 'track_semantic_object'
            )
            try:
                if tool_name == 'get_semantic_objects':
                    result = self.get_semantic_objects(args.get('class_name', ''))
                elif tool_name == 'semantic_object_navigation':
                    result = self.semantic_object_navigation(
                        args.get('object_name', ''),
                        args.get('selection', 'nearest'),
                    )
                elif tool_name == 'return_to_origin':
                    result = self.return_to_origin()
                elif tool_name == 'track_semantic_object':
                    result = self.track_semantic_object(args.get('object_name', ''))
                elif tool_name == 'stop_tracking':
                    result = self.stop_tracking()
                else:
                    result = f'Unknown semantic tool: {tool_name}'
            except Exception as e:
                self.get_logger().error(f'Tool {tool_name} failed: {e}')
                result = f'Tool {tool_name} failed: {e}'

            self.tools_result_pub.publish(Tools(id=tool_id, name=tool_name, data=result))
            self.task_running = False

    def _speak(self, text):
        if not text:
            return
        msg = String()
        msg.data = str(text)
        self.tts_text_pub.publish(msg)

    def _enable_wakeup(self, enabled):
        if not self.awake_client.service_is_ready():
            return
        awake = SetBool.Request()
        awake.data = bool(enabled)
        self.awake_client.call_async(awake)

    def _cancel_navigation(self):
        if self.cancel_nav_client.service_is_ready():
            self.send_request(self.cancel_nav_client, Trigger.Request())

    def get_semantic_objects(self, class_name=''):
        if class_name and self._is_origin_name(class_name):
            return self._origin_description()

        objects, source = self._load_objects()
        if class_name:
            target = self._canonical_name(class_name)
            named_objects = self._named_objects(objects, class_name)
            objects = named_objects or [
                obj for obj in objects if self._object_matches(obj, target, class_name)
            ]

        if not objects:
            if LANGUAGE == 'Chinese':
                missing_name = class_name or '目标'
                return f'\u8bed\u4e49\u5730\u56fe\u4e2d\u6ca1\u6709\u627e\u5230\u201c{missing_name}\u201d\u3002'
            return 'No matching semantic objects are known yet.'

        self.get_logger().info(f'Listing {len(objects)} semantic objects from {source}')
        objects = sorted(objects, key=lambda obj: obj.get('last_seen', 0.0), reverse=True)
        if LANGUAGE == 'Chinese':
            counts = {}
            for obj in objects:
                label = _class_name_zh(obj.get('class_name', ''))
                counts[label] = counts.get(label, 0) + 1
            summary = '\uff0c'.join(f'{name}{count}\u4e2a' for name, count in counts.items())
            details = []
            for obj in objects[:10]:
                pos = obj.get('position', [0.0, 0.0, 0.0])
                label = obj.get('display_name') or _class_name_zh(obj.get('class_name', ''))
                details.append(f'{label}\uff08{obj.get("id")}\uff0cx={pos[0]:.2f}\uff0cy={pos[1]:.2f}\uff09')
            details_text = '；'.join(details)
            origin_text = self._origin_description() if not class_name else ''
            return (
                f'\u8bed\u4e49\u5730\u56fe\u4e2d\u5171\u6709{len(objects)}\u4e2a\u76ee\u6807\uff1a{summary}\u3002'
                f'\u70b9\u4f4d\uff1a{details_text}\u3002{origin_text}'
            )

        lines = []
        for obj in objects[:20]:
            pos = obj.get('position', [0.0, 0.0, 0.0])
            display_name = obj.get('display_name')
            display_text = f" named {display_name}" if display_name else ''
            lines.append(
                f"{obj.get('id')} {obj.get('class_name')}{display_text} at "
                f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
                f"obs={obj.get('observations', 0)}, conf={obj.get('confidence', 0.0):.2f}"
            )
        return '\n'.join(lines)

    def semantic_object_navigation(self, object_name, selection='nearest'):
        if self._is_origin_name(object_name):
            return self.return_to_origin()

        self._wait_for_current_pose()
        obj = self._select_object(object_name, selection)
        if obj is None:
            if LANGUAGE == 'Chinese':
                return f'\u8bed\u4e49\u5730\u56fe\u4e2d\u6ca1\u6709\u627e\u5230\u53ef\u9760\u7684\u201c{object_name}\u201d\u76ee\u6807\u3002'
            return f"I have not mapped a reliable {object_name} yet."

        sent_any_goal = False
        candidates = self._approach_candidates_for_object(obj)
        safe_candidates = [candidate for candidate in candidates if candidate['safe']]
        if safe_candidates:
            candidates = safe_candidates
        elif self.nav_map is not None:
            self.get_logger().warn(
                f'No safe free-space approach pose found near {obj.get("id")} on {self.map_topic}'
            )
            return f"No safe navigation pose near {obj.get('class_name')} ({obj.get('id')})."

        candidates = self._rank_candidates_by_planned_path(candidates)

        for candidate in candidates[:self.max_goal_attempts]:
            x = candidate['x']
            y = candidate['y']
            yaw_deg = candidate['yaw_deg']
            self.reach_goal = False
            self.navigation_done = False
            if not self._send_goal(x, y, yaw_deg):
                continue

            sent_any_goal = True
            if LANGUAGE == 'Chinese':
                self._speak(f'\u6536\u5230\uff0c\u6b63\u5728\u524d\u5f80{object_name}\u9644\u8fd1')
            attempt_result = self._wait_for_navigation_attempt(candidate)

            if attempt_result == 'canceled':
                return f"Canceled navigation to {obj.get('class_name')} ({obj.get('id')})."
            if attempt_result in ('reached', 'near_goal'):
                return f"Arrived near {obj.get('class_name')} ({obj.get('id')})."
            if attempt_result in ('failed', 'stalled', 'timeout'):
                self.get_logger().warn(
                    'Navigation %s at stand_off=%.2f goal=(%.2f, %.2f) near %s; trying next candidate'
                    % (attempt_result, candidate['stand_off'], x, y, obj.get('id'))
                )
                if attempt_result == 'timeout':
                    self._cancel_navigation()
                continue
            return f"Navigation goal sent near {obj.get('class_name')} ({obj.get('id')}), still waiting or timed out."

        if sent_any_goal:
            return f"Navigation failed near {obj.get('class_name')} ({obj.get('id')}) after trying safer approach distances."
        return f"Failed to send navigation goal near {obj.get('class_name')} ({obj.get('id')})."

    def _wait_for_navigation_attempt(self, candidate):
        start = time.time()
        last_motion_time = start
        last_pose = self._current_xy()

        while (
            rclpy.ok()
            and not self.interrupt
            and not self.navigation_done
            and (time.time() - start) < self.navigation_timeout
        ):
            if self._is_near_candidate_goal(candidate):
                self.get_logger().info(
                    'Semantic goal accepted near candidate: goal=(%.2f, %.2f) distance<=%.2f'
                    % (candidate['x'], candidate['y'], self.accept_near_goal_distance)
                )
                self._cancel_navigation()
                return 'near_goal'

            current_pose = self._current_xy()
            if current_pose is not None:
                if last_pose is None:
                    last_pose = current_pose
                    last_motion_time = time.time()
                elif self._distance(current_pose, last_pose) >= self.stall_min_movement:
                    last_pose = current_pose
                    last_motion_time = time.time()
                elif self.retry_on_stall and time.time() - last_motion_time > self.stall_timeout:
                    self.get_logger().warn(
                        'Semantic navigation appears stalled: goal=(%.2f, %.2f), '
                        'no %.2fm translation for %.1fs; canceling this candidate'
                        % (
                            candidate['x'],
                            candidate['y'],
                            self.stall_min_movement,
                            self.stall_timeout,
                        )
                    )
                    self._cancel_navigation()
                    return 'stalled'

            time.sleep(0.1)

        if self.interrupt:
            return 'canceled'
        if self.reach_goal:
            return 'reached'
        if self.navigation_done:
            return 'failed'
        return 'timeout'

    def _current_xy(self):
        if self.current_pose is None:
            return None
        return self.current_pose['x'], self.current_pose['y']

    def _is_near_candidate_goal(self, candidate):
        current_pose = self._current_xy()
        if current_pose is None:
            return False
        return self._distance(current_pose, (candidate['x'], candidate['y'])) <= self.accept_near_goal_distance

    def return_to_origin(self):
        if self.nav_map is not None and not self._is_goal_safe(self.origin_x, self.origin_y):
            self.get_logger().warn(
                'Origin goal is not in safe free space: x=%.2f, y=%.2f'
                % (self.origin_x, self.origin_y)
            )
            if LANGUAGE == 'Chinese':
                return '原点位置当前不在可安全通行区域，无法返回。'
            return 'The origin is not currently a safe navigation goal.'

        self.reach_goal = False
        self.navigation_done = False
        if not self._send_goal(self.origin_x, self.origin_y, self.origin_yaw):
            if LANGUAGE == 'Chinese':
                return '返回原点的导航目标发送失败。'
            return 'Failed to send the return-to-origin navigation goal.'

        if LANGUAGE == 'Chinese':
            self._speak('收到，正在返回原点')
        start = time.time()
        while (
            rclpy.ok()
            and not self.interrupt
            and not self.navigation_done
            and (time.time() - start) < self.navigation_timeout
        ):
            time.sleep(0.1)

        if self.interrupt:
            return '已取消返回原点。' if LANGUAGE == 'Chinese' else 'Canceled return to origin.'
        if self.reach_goal:
            return '已回到原点。' if LANGUAGE == 'Chinese' else 'Arrived at the origin.'
        if self.navigation_done:
            return '返回原点失败。' if LANGUAGE == 'Chinese' else 'Navigation to origin failed.'
        return '已发送返回原点目标，仍在等待导航结果。' if LANGUAGE == 'Chinese' else 'Return-to-origin goal is still pending.'

    def track_semantic_object(self, object_name):
        self.tracking_target = object_name
        self.last_tracking_goal = None
        if self.tracking_thread is None or not self.tracking_thread.is_alive():
            self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
            self.tracking_thread.start()
        return f"Started semantic tracking for {object_name}."

    def stop_tracking(self):
        self.tracking_target = None
        self._cancel_navigation()
        return 'Stopped semantic object tracking.'

    def _tracking_loop(self):
        while rclpy.ok():
            if self.interrupt:
                self.tracking_target = None
                time.sleep(0.2)
                continue
            target = self.tracking_target
            if not target:
                time.sleep(0.2)
                continue
            obj = self._select_object(target, 'latest')
            if obj is None:
                time.sleep(self.tracking_period)
                continue
            if time.time() - float(obj.get('last_seen', 0.0)) > self.lost_timeout:
                time.sleep(self.tracking_period)
                continue

            candidate = self._best_approach_candidate_for_object(obj)
            if candidate is None:
                time.sleep(self.tracking_period)
                continue
            x, y, yaw_deg = candidate['x'], candidate['y'], candidate['yaw_deg']
            goal = (x, y)
            if self.last_tracking_goal is None or self._distance(goal, self.last_tracking_goal) > self.goal_update_distance:
                if self._send_goal(x, y, yaw_deg):
                    self.last_tracking_goal = goal
            time.sleep(self.tracking_period)

    def _load_objects(self):
        if self.live_objects and time.time() - self.live_objects_updated_at <= self.live_objects_timeout:
            return list(self.live_objects), 'live_topic'

        if not os.path.exists(self.map_file):
            return [], 'missing_file'
        try:
            with open(self.map_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            return data.get('objects', []), 'map_file'
        except Exception as e:
            self.get_logger().warn(f'Failed to read semantic map: {e}', throttle_duration_sec=2.0)
            return [], 'map_file_error'

    def _select_object(self, object_name, selection):
        target = self._canonical_name(object_name)
        objects, source = self._load_objects()
        named_candidates = self._named_objects(objects, object_name)
        candidates = named_candidates or [
            obj for obj in objects if self._object_matches(obj, target, object_name)
        ]
        if not candidates:
            known = sorted({str(obj.get('class_name', '')) for obj in objects if obj.get('class_name')})
            self.get_logger().warn(
                f'No semantic candidate for "{object_name}" -> "{target}" from {source}. '
                f'Known classes: {known[:20]}'
            )
            return None

        stable_candidates = [
            obj for obj in candidates
            if int(obj.get('observations', 0)) >= self.min_observations
        ]
        if stable_candidates:
            candidates = stable_candidates
        else:
            self.get_logger().warn(
                f'No stable candidate for "{target}" with min_observations={self.min_observations}; '
                f'falling back to {len(candidates)} raw candidates'
            )

        selection = selection or 'nearest'
        if selection == 'latest':
            selected = max(candidates, key=lambda obj: obj.get('last_seen', 0.0))
        elif selection == 'most_confident':
            selected = max(candidates, key=lambda obj: obj.get('confidence', 0.0))
        elif self.current_pose is None:
            selected = max(candidates, key=lambda obj: obj.get('last_seen', 0.0))
        else:
            selected = min(candidates, key=lambda obj: self._distance_to_robot(obj))

        self._log_selection(object_name, target, selection, source, candidates, selected)
        return selected

    def _canonical_name(self, name):
        stripped = str(name or '').strip()
        if stripped in CLASS_ALIASES:
            return _normalize(CLASS_ALIASES[stripped])

        compact = stripped.replace(' ', '')
        for word in ATTRIBUTE_WORDS:
            compact = compact.replace(word, '')
        compact = compact.replace('\u7684', '')

        if compact in CLASS_ALIASES:
            return _normalize(CLASS_ALIASES[compact])

        for alias, class_name in sorted(CLASS_ALIASES.items(), key=lambda item: len(item[0]), reverse=True):
            if alias and alias in stripped:
                return _normalize(class_name)

        return _normalize(compact or stripped)

    def _object_matches(self, obj, target, requested_name=''):
        class_name = _normalize(obj.get('class_name', ''))
        object_id = _normalize(obj.get('id', ''))
        display_name = _normalize(obj.get('display_name', ''))
        display_target = self._canonical_name(display_name) if display_name else ''
        requested_name = _normalize(requested_name)
        return (
            target in class_name
            or class_name in target
            or target in object_id
            or (display_target and (target in display_target or display_target in target))
            or (display_name and (target in display_name or display_name in target))
            or (
                display_name
                and requested_name
                and (requested_name in display_name or display_name in requested_name)
            )
        )

    def _named_objects(self, objects, requested_name):
        requested_name = _normalize(requested_name)
        if not requested_name:
            return []
        requested_target = self._canonical_name(requested_name)
        return [
            obj for obj in objects
            if (
                _normalize(obj.get('display_name', '')) == requested_name
                or (
                    obj.get('display_name')
                    and self._canonical_name(obj.get('display_name')) == requested_target
                )
            )
        ]

    def _is_origin_name(self, name):
        normalized = _normalize(name)
        compact = normalized.replace(' ', '')
        if normalized in ORIGIN_NAMES or compact in ORIGIN_NAMES:
            return True
        return any(
            alias in compact
            for alias in ORIGIN_NAMES
            if any('\u4e00' <= char <= '\u9fff' for char in alias)
        )

    def _origin_description(self):
        if LANGUAGE == 'Chinese':
            return (
                f'\u56fa\u5b9a\u8fd4\u56de\u70b9\uff1a\u539f\u70b9'
                f'\uff08x={self.origin_x:.2f}\uff0cy={self.origin_y:.2f}\uff0c'
                f'\u671d\u5411={self.origin_yaw:.1f}\u5ea6\uff09\u3002'
            )
        return (
            f'Origin: x={self.origin_x:.2f}, y={self.origin_y:.2f}, '
            f'yaw={self.origin_yaw:.1f} degrees.'
        )

    def _distance_to_robot(self, obj):
        pos = obj.get('position', [0.0, 0.0, 0.0])
        return math.hypot(pos[0] - self.current_pose['x'], pos[1] - self.current_pose['y'])

    def _log_selection(self, object_name, target, selection, source, candidates, selected):
        details = []
        for obj in candidates[:6]:
            pos = obj.get('position', [0.0, 0.0, 0.0])
            label = obj.get('display_name') or obj.get('id')
            if self.current_pose is None:
                dist_text = 'dist=?'
            else:
                dist_text = f'dist={self._distance_to_robot(obj):.2f}'
            details.append(
                f"{label}:{obj.get('class_name')} "
                f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                f"obs={obj.get('observations', 0)} conf={obj.get('confidence', 0.0):.2f} {dist_text}"
            )
        selected_pos = selected.get('position', [0.0, 0.0, 0.0])
        self.get_logger().info(
            f'Semantic target "{object_name}" -> "{target}", selection={selection}, source={source}, '
            f'candidates={len(candidates)}, selected={selected.get("display_name") or selected.get("id")} '
            f'({selected.get("class_name")}) at ({selected_pos[0]:.2f}, {selected_pos[1]:.2f}, {selected_pos[2]:.2f}); '
            f'candidates: {"; ".join(details)}'
        )

    def _wait_for_current_pose(self, timeout_sec=3.0):
        start = time.time()
        while rclpy.ok() and self.current_pose is None and time.time() - start < timeout_sec:
            time.sleep(0.05)
        if self.current_pose is None:
            self.get_logger().warn('No /amcl_pose received yet; semantic goal will use fallback approach direction')

    def _goal_retry_distances(self):
        distances = [max(0.05, self.stand_off_distance), 0.30, 0.45, 0.60, 0.90, 1.20]
        result = []
        for distance in distances:
            distance = round(float(distance), 2)
            if all(abs(distance - existing) > 0.04 for existing in result):
                result.append(distance)
        return result

    def _best_approach_candidate_for_object(self, obj):
        candidates = self._approach_candidates_for_object(obj)
        for candidate in candidates:
            if candidate['safe'] or self.nav_map is None:
                return candidate
        return None

    def _approach_candidates_for_object(self, obj):
        pos = obj.get('position', [0.0, 0.0, 0.0])
        obj_x, obj_y = float(pos[0]), float(pos[1])

        if self.current_pose is not None:
            base_angle = math.atan2(self.current_pose['y'] - obj_y, self.current_pose['x'] - obj_x)
        else:
            base_angle = math.pi

        candidates = []
        for distance_index, stand_off in enumerate(self.goal_retry_distances):
            for angle_index, offset in enumerate(self._approach_angle_offsets()):
                angle = base_angle + offset
                x = obj_x + stand_off * math.cos(angle)
                y = obj_y + stand_off * math.sin(angle)
                yaw = self._candidate_goal_yaw(obj_x, obj_y, x, y)
                safe = self._is_goal_safe(x, y)
                if self.current_pose is None:
                    robot_dist = 0.0
                else:
                    robot_dist = math.hypot(x - self.current_pose['x'], y - self.current_pose['y'])
                candidates.append({
                    'x': x,
                    'y': y,
                    'yaw_deg': math.degrees(yaw),
                    'stand_off': stand_off,
                    'safe': safe,
                    'distance_index': distance_index,
                    'angle_offset': abs(offset),
                    'robot_dist': robot_dist,
                    'angle_index': angle_index,
                })

        candidates.sort(key=lambda candidate: (
            not candidate['safe'],
            candidate['distance_index'],
            candidate['angle_offset'],
            candidate['robot_dist'],
            candidate['angle_index'],
        ))
        if candidates:
            best = candidates[0]
            self.get_logger().info(
                'Best semantic approach candidate: id=%s goal=(%.2f, %.2f) stand_off=%.2f safe=%s map=%s'
                % (
                    obj.get('id'),
                    best['x'],
                    best['y'],
                    best['stand_off'],
                    best['safe'],
                    'ok' if self.nav_map is not None else 'missing',
                )
            )
            if best['safe'] and best['stand_off'] > self.stand_off_distance + 0.04:
                self.get_logger().warn(
                    'Requested stand_off=%.2f has no safe approach pose; using fallback stand_off=%.2f near %s'
                    % (self.stand_off_distance, best['stand_off'], obj.get('id'))
                )
        return candidates

    def _approach_angle_offsets(self):
        step = 2.0 * math.pi / float(self.approach_angle_samples)
        offsets = [0.0]
        for i in range(1, self.approach_angle_samples // 2 + 1):
            offsets.append(i * step)
            offsets.append(-i * step)
        return offsets[:self.approach_angle_samples]

    def _candidate_goal_yaw(self, obj_x, obj_y, goal_x, goal_y):
        if self.goal_yaw_mode == 'face_target':
            return math.atan2(obj_y - goal_y, obj_x - goal_x)
        if self.goal_yaw_mode == 'keep_current' and self.current_pose is not None:
            return self.current_pose['yaw']
        if self.current_pose is not None:
            return math.atan2(goal_y - self.current_pose['y'], goal_x - self.current_pose['x'])
        return math.atan2(obj_y - goal_y, obj_x - goal_x)

    def _rank_candidates_by_planned_path(self, candidates):
        if not self.enable_path_scoring or self.nav_map is None or not candidates:
            return candidates
        if not self.compute_path_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f'Planner action {self.planner_action_name} is not available; using geometric candidate order'
            )
            return candidates

        limit = min(len(candidates), self.max_path_score_candidates)
        scored = []
        scored_indices = set()
        for index, candidate in enumerate(candidates[:limit]):
            path = self._compute_path_to_candidate(candidate)
            if path is None or not path.poses:
                continue

            path_length = self._path_length(path)
            path_turning = self._path_turning_score(path)
            clearance = self._path_min_clearance(path)
            side_margin = clearance - self.vehicle_width * 0.5
            margin_deficit = max(0.0, self.preferred_obstacle_margin - side_margin)
            scored_candidate = dict(candidate)
            scored_candidate.update({
                'path_length': path_length,
                'path_turning': path_turning,
                'path_min_clearance': clearance,
                'path_side_margin': side_margin,
                'path_margin_deficit': margin_deficit,
                'path_score': (
                    0 if side_margin >= self.preferred_obstacle_margin else 1,
                    round(margin_deficit, 4),
                    -round(min(side_margin, self.preferred_obstacle_margin + 0.12), 4),
                    round(path_length, 3),
                    round(path_turning, 3),
                    candidate['distance_index'],
                    round(candidate['angle_offset'], 3),
                ),
            })
            scored.append(scored_candidate)
            scored_indices.add(index)

        if not scored:
            self.get_logger().warn('No candidate path could be precomputed; using geometric candidate order')
            return candidates

        scored.sort(key=lambda candidate: candidate['path_score'])
        best = scored[0]
        self.get_logger().info(
            'Best planned semantic candidate: goal=(%.2f, %.2f) stand_off=%.2f '
            'path_len=%.2f turning=%.2f center_clearance=%.2f side_margin=%.2f preferred_margin=%.2f'
            % (
                best['x'],
                best['y'],
                best['stand_off'],
                best['path_length'],
                best['path_turning'],
                best['path_min_clearance'],
                best['path_side_margin'],
                self.preferred_obstacle_margin,
            )
        )

        unscored = [
            candidate for index, candidate in enumerate(candidates)
            if index not in scored_indices
        ]
        return scored + unscored

    def _compute_path_to_candidate(self, candidate):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._make_pose_stamped(candidate['x'], candidate['y'], candidate['yaw_deg'])
        if hasattr(goal_msg, 'use_start'):
            goal_msg.use_start = False
        if hasattr(goal_msg, 'planner_id'):
            goal_msg.planner_id = ''

        goal_future = self.compute_path_client.send_goal_async(goal_msg)
        goal_handle = self._wait_for_future(goal_future, self.path_scoring_timeout)
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, self.path_scoring_timeout)
        if result is None or result.result is None:
            return None
        return result.result.path

    def _wait_for_future(self, future, timeout_sec):
        start_time = time.time()
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().warn(f'Future failed while scoring semantic path: {e}')
                    return None
            if time.time() - start_time > timeout_sec:
                return None
            time.sleep(0.01)
        return None

    def _make_pose_stamped(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        yaw = math.radians(float(yaw_deg))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _path_length(self, path):
        poses = path.poses
        if len(poses) < 2:
            return 0.0
        total = 0.0
        prev = poses[0].pose.position
        for stamped in poses[1:]:
            pos = stamped.pose.position
            total += math.hypot(pos.x - prev.x, pos.y - prev.y)
            prev = pos
        return total

    def _path_turning_score(self, path):
        poses = path.poses
        if len(poses) < 3:
            return 0.0

        headings = []
        prev = poses[0].pose.position
        for stamped in poses[1:]:
            pos = stamped.pose.position
            step = math.hypot(pos.x - prev.x, pos.y - prev.y)
            if step >= 0.03:
                headings.append(math.atan2(pos.y - prev.y, pos.x - prev.x))
                prev = pos

        if len(headings) < 2:
            return 0.0

        total = 0.0
        for prev_heading, heading in zip(headings, headings[1:]):
            total += abs(math.atan2(
                math.sin(heading - prev_heading),
                math.cos(heading - prev_heading),
            ))
        return total

    def _path_min_clearance(self, path):
        if self.nav_map is None:
            return self.max_clearance_check
        poses = path.poses
        if not poses:
            return 0.0

        sample_step = max(1, len(poses) // 80)
        samples = list(poses[::sample_step])
        if samples[-1] is not poses[-1]:
            samples.append(poses[-1])

        info = self.nav_map.info
        min_clearance = self.max_clearance_check
        for stamped in samples:
            pos = stamped.pose.position
            cell = self._world_to_map(pos.x, pos.y)
            if cell is None:
                return 0.0
            clearance = self._cell_clearance(info, cell[0], cell[1])
            min_clearance = min(min_clearance, clearance)
            if min_clearance <= 0.0:
                return 0.0
        return min_clearance

    def _cell_clearance(self, info, mx, my):
        if self._is_blocking_cell(info, mx, my):
            return 0.0

        max_cells = int(math.ceil(self.max_clearance_check / max(info.resolution, 1e-6)))
        for radius in range(1, max_cells + 1):
            best = None
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cx = mx + dx
                    cy = my + dy
                    if self._is_blocking_cell(info, cx, cy):
                        dist = math.hypot(dx, dy) * info.resolution
                        best = dist if best is None else min(best, dist)
            if best is not None:
                return min(best, self.max_clearance_check)
        return self.max_clearance_check

    def _is_blocking_cell(self, info, mx, my):
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return True
        value = self.nav_map.data[my * info.width + mx]
        if value < 0:
            return self.path_treat_unknown_as_obstacle
        return value >= self.occupancy_threshold

    def _is_goal_safe(self, x, y):
        if self.nav_map is None:
            return True
        cell = self._world_to_map(x, y)
        if cell is None:
            return False
        mx, my = cell
        info = self.nav_map.info
        radius = int(math.ceil(self.goal_clearance / max(info.resolution, 1e-6)))
        radius_sq = radius * radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                cx = mx + dx
                cy = my + dy
                if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
                    return False
                value = self.nav_map.data[cy * info.width + cx]
                if value < 0:
                    if not self.allow_unknown_goals:
                        return False
                    continue
                if value >= self.occupancy_threshold:
                    return False
        return True

    def _world_to_map(self, x, y):
        if self.nav_map is None:
            return None
        info = self.nav_map.info
        origin = info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        mx = int(math.floor(local_x / info.resolution))
        my = int(math.floor(local_y / info.resolution))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def _yaw_from_quaternion(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _approach_pose_for_object(self, obj, stand_off_distance=None):
        pos = obj.get('position', [0.0, 0.0, 0.0])
        obj_x, obj_y = float(pos[0]), float(pos[1])
        stand_off_distance = self.stand_off_distance if stand_off_distance is None else float(stand_off_distance)

        if self.current_pose is not None:
            dx = self.current_pose['x'] - obj_x
            dy = self.current_pose['y'] - obj_y
        else:
            dx, dy = -1.0, 0.0

        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            dx, dy, norm = -1.0, 0.0, 1.0

        approach_x = obj_x + stand_off_distance * dx / norm
        approach_y = obj_y + stand_off_distance * dy / norm
        yaw = math.atan2(obj_y - approach_y, obj_x - approach_x)
        return approach_x, approach_y, math.degrees(yaw)

    def _send_goal(self, x, y, yaw_deg):
        request = SetPose2D.Request()
        request.data.x = float(x)
        request.data.y = float(y)
        request.data.roll = 0.0
        request.data.pitch = 0.0
        request.data.yaw = float(yaw_deg)
        response = self.send_request(self.set_pose_client, request)
        if response is None:
            self.get_logger().warn(f'Failed to send semantic nav goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}')
            return False
        if hasattr(response, 'success') and not response.success:
            self.get_logger().warn(f'Semantic nav goal rejected: {getattr(response, "message", "")}')
            return False
        self.get_logger().info(f'Sent semantic nav goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}')
        return True

    def _distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigationTool()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.tracking_target = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
