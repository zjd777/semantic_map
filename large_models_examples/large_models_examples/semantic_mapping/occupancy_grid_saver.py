#!/usr/bin/env python3
# encoding: utf-8

import json
import math
import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _expand_path(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path)))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SemanticOccupancyGridSaver(Node):
    """Save /map as a map_server map, optionally overlaying semantic objects as occupied cells."""

    def __init__(self):
        super().__init__('semantic_occupancy_grid_saver')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_prefix', '~/ros2_ws/src/slam/maps/semantic_map')
        self.declare_parameter('semantic_overlay_enabled', False)
        self.declare_parameter('semantic_objects_topic', '/semantic_map/objects')
        self.declare_parameter('semantic_obstacle_radius', 0.18)
        self.declare_parameter('semantic_min_observations_for_occupancy', 2)
        self.declare_parameter('semantic_min_confidence_for_occupancy', 0.60)
        self.declare_parameter('semantic_occupancy_classes', 'all')

        self.map_topic = self.get_parameter('map_topic').value
        self.output_prefix = _expand_path(self.get_parameter('output_prefix').value)
        self.semantic_overlay_enabled = _as_bool(self.get_parameter('semantic_overlay_enabled').value)
        self.semantic_objects_topic = self.get_parameter('semantic_objects_topic').value
        self.semantic_obstacle_radius = max(0.0, float(self.get_parameter('semantic_obstacle_radius').value))
        self.semantic_min_observations = max(
            1, int(self.get_parameter('semantic_min_observations_for_occupancy').value)
        )
        self.semantic_min_confidence = float(self.get_parameter('semantic_min_confidence_for_occupancy').value)
        self.semantic_occupancy_classes = self._parse_class_filter(
            self.get_parameter('semantic_occupancy_classes').value
        )

        self.latest_map = None
        self.semantic_objects = []
        self.semantic_objects_frame = ''

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos)
        self.create_subscription(String, self.semantic_objects_topic, self.semantic_objects_callback, 1)
        self.create_service(Trigger, '~/save', self.save_callback)

        self.get_logger().info(
            f'Semantic occupancy grid saver listening on {self.map_topic}; '
            f'semantic_overlay={self.semantic_overlay_enabled} '
            f'semantic_topic={self.semantic_objects_topic}'
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def semantic_objects_callback(self, msg):
        try:
            data = json.loads(msg.data or '{}')
        except Exception as e:
            self.get_logger().warn(
                f'Failed to parse semantic objects for 2D overlay: {e}',
                throttle_duration_sec=2.0,
            )
            return

        objects = data.get('objects', [])
        if not isinstance(objects, list):
            return

        self.semantic_objects = objects
        self.semantic_objects_frame = data.get('frame_id', '')

    def save_callback(self, request, response):
        if self.latest_map is None:
            response.success = False
            response.message = f'No occupancy grid received on {self.map_topic}'
            return response

        try:
            semantic_count, semantic_cell_count = self._save_map()
        except Exception as e:
            response.success = False
            response.message = f'Failed to save occupancy grid: {e}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'Saved {self.output_prefix}.pgm and {self.output_prefix}.yaml '
            f'with semantic_overlay_objects={semantic_count}, '
            f'semantic_overlay_cells={semantic_cell_count}'
        )
        self.get_logger().info(response.message)
        return response

    def _save_map(self):
        msg = self.latest_map
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        data = list(msg.data)

        semantic_count, semantic_cell_count = self._overlay_semantic_occupancy(msg, data)

        output_dir = os.path.dirname(self.output_prefix)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        pgm_path = self.output_prefix + '.pgm'
        yaml_path = self.output_prefix + '.yaml'

        with open(pgm_path, 'wb') as file:
            header = f'P5\n# CREATOR: semantic_occupancy_grid_saver\n{width} {height}\n255\n'
            file.write(header.encode('ascii'))
            for y in range(height - 1, -1, -1):
                row = bytearray()
                for x in range(width):
                    value = int(data[y * width + x])
                    if value < 0:
                        row.append(205)
                    elif value >= 65:
                        row.append(0)
                    else:
                        row.append(254)
                file.write(row)

        origin = msg.info.origin.position
        yaw = _yaw_from_quat(msg.info.origin.orientation)
        image_name = os.path.basename(pgm_path)
        with open(yaml_path, 'w', encoding='utf-8') as file:
            file.write(
                f'image: {image_name}\n'
                f'mode: trinary\n'
                f'resolution: {resolution:.6f}\n'
                f'origin: [{origin.x:.6f}, {origin.y:.6f}, {yaw:.6f}]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n'
            )

        self.get_logger().info(f'Saved 2D occupancy map: {yaml_path}')
        return semantic_count, semantic_cell_count

    def _overlay_semantic_occupancy(self, msg, data):
        if not self.semantic_overlay_enabled or not self.semantic_objects:
            return 0, 0

        info = msg.info
        map_frame = msg.header.frame_id or 'map'
        if self.semantic_objects_frame and self.semantic_objects_frame != map_frame:
            self.get_logger().warn(
                f'Semantic object frame {self.semantic_objects_frame} does not match map frame {map_frame}; '
                'skip semantic 2D overlay.',
                throttle_duration_sec=2.0,
            )
            return 0, 0

        radius_cells = max(0, int(math.ceil(self.semantic_obstacle_radius / max(info.resolution, 1e-6))))
        occupied_objects = 0
        occupied_cells = set()

        for obj in self.semantic_objects:
            if not self._semantic_object_should_occupy(obj):
                continue

            pos = obj.get('position', [])
            if not isinstance(pos, list) or len(pos) < 2:
                continue

            cell = self._world_to_map(info, float(pos[0]), float(pos[1]))
            if cell is None:
                continue

            mx, my = cell
            object_marked = False
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    cx = mx + dx
                    cy = my + dy
                    if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
                        continue
                    index = cy * info.width + cx
                    data[index] = 100
                    occupied_cells.add(index)
                    object_marked = True

            if object_marked:
                occupied_objects += 1

        return occupied_objects, len(occupied_cells)

    def _semantic_object_should_occupy(self, obj):
        class_name = str(obj.get('class_name', '')).strip().lower().replace('_', ' ')
        if not class_name:
            return False
        if self.semantic_occupancy_classes and class_name not in self.semantic_occupancy_classes:
            return False
        if int(obj.get('observations', 0)) < self.semantic_min_observations:
            return False
        if float(obj.get('confidence', 0.0)) < self.semantic_min_confidence:
            return False
        return True

    def _world_to_map(self, info, x, y):
        origin = info.origin
        yaw = _yaw_from_quat(origin.orientation)
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

    def _parse_class_filter(self, value):
        text_value = str(value or '').strip().lower()
        if text_value in ('', 'all', '*', 'none'):
            return set()
        if isinstance(value, (list, tuple)):
            names = value
        else:
            names = str(value or '').split(',')
        return {
            str(name).strip().lower().replace('_', ' ')
            for name in names
            if str(name).strip()
        }


def main(args=None):
    rclpy.init(args=args)
    node = SemanticOccupancyGridSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
