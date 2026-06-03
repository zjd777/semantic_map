#!/usr/bin/env python3
# encoding: utf-8

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _expand_path(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path)))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _class_color(class_name):
    if class_name == 'occupied':
        return 0.55, 0.58, 0.62, 0.16

    palette = [
        (0.0, 0.85, 0.35),
        (0.1, 0.65, 1.0),
        (1.0, 0.75, 0.15),
        (1.0, 0.35, 0.35),
        (0.65, 0.45, 1.0),
        (0.2, 0.95, 0.9),
    ]
    idx = sum(ord(ch) for ch in class_name) % len(palette)
    r, g, b = palette[idx]
    return r, g, b, 0.34


class SemanticMapMarkerPublisher(Node):
    """Publish saved semantic objects and voxels for RViz without updating the map."""

    def __init__(self):
        super().__init__('semantic_map_marker_publisher')

        self.declare_parameter('map_file', '~/.ros/semantic_voxel_map.json')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('publish_voxel_markers', True)
        self.declare_parameter('max_marker_voxels', 700)
        self.declare_parameter('min_occupied_count_for_marker', 2)
        self.declare_parameter('publish_origin_marker', True)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)

        self.map_file = _expand_path(self.get_parameter('map_file').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_period = max(0.1, float(self.get_parameter('publish_period').value))
        self.publish_voxel_markers = _as_bool(self.get_parameter('publish_voxel_markers').value)
        self.max_marker_voxels = int(self.get_parameter('max_marker_voxels').value)
        self.min_occupied_count_for_marker = max(1, int(self.get_parameter('min_occupied_count_for_marker').value))
        self.publish_origin_marker = _as_bool(self.get_parameter('publish_origin_marker').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_yaw = float(self.get_parameter('origin_yaw').value)

        self.objects = []
        self.voxels = []
        self.voxel_size = 0.08
        self.last_mtime = None
        self.last_status_log = 0.0

        self.summary_pub = self.create_publisher(String, '/semantic_map/objects', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map/markers', 1)
        self.create_timer(self.publish_period, self.periodic_publish)

        self.get_logger().info(f'Saved semantic marker publisher started from {self.map_file}')

    def _load_if_changed(self):
        if not os.path.exists(self.map_file):
            return False

        mtime = os.path.getmtime(self.map_file)
        if self.last_mtime == mtime:
            return True

        try:
            with open(self.map_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to read saved semantic map {self.map_file}: {e}', throttle_duration_sec=2.0)
            return False

        self.map_frame = data.get('frame_id') or self.map_frame
        self.voxel_size = float(data.get('voxel_size', self.voxel_size))
        self.objects = data.get('objects', [])
        self.voxels = data.get('voxels', [])
        self.last_mtime = mtime
        self.get_logger().info(
            f'Loaded saved semantic map markers: objects={len(self.objects)} voxels={len(self.voxels)} frame={self.map_frame}'
        )
        return True

    def periodic_publish(self):
        loaded = self._load_if_changed()
        self._publish_summary()
        self._publish_markers()

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_status_log > 5.0:
            self.last_status_log = now
            if loaded:
                self.get_logger().info(
                    f'Saved semantic markers: objects={len(self.objects)} voxels={len(self.voxels)} frame={self.map_frame}'
                )
            else:
                self.get_logger().warn(f'Saved semantic map not found yet: {self.map_file}')

    def _publish_summary(self):
        data = {
            'frame_id': self.map_frame,
            'updated_at': time.time(),
            'object_count': len(self.objects),
            'objects': self.objects,
        }
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.summary_pub.publish(msg)

    def _publish_markers(self):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        stamp = self.get_clock().now().to_msg()
        marker_id = 1
        if self.publish_origin_marker:
            marker_id = self._append_origin_markers(markers, stamp, marker_id)

        for obj in self.objects:
            pos = obj.get('position', [0.0, 0.0, 0.0])

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'semantic_objects'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color.r = 0.1
            sphere.color.g = 0.7
            sphere.color.b = 1.0
            sphere.color.a = 0.85
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = stamp
            text.ns = 'semantic_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = float(pos[2]) + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.16
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            label = obj.get('display_name') or obj.get('id') or obj.get('class_name', 'object')
            text.text = f"{label} [{obj.get('class_name', 'object')}]"
            markers.markers.append(text)

        if self.publish_voxel_markers:
            visible_voxels = self._visible_voxels()
            for voxel in visible_voxels:
                key = voxel.get('key', [])
                if len(key) != 3:
                    continue
                class_counts = voxel.get('class_counts', {})
                semantic_counts = {
                    name: count for name, count in class_counts.items()
                    if name != 'occupied'
                }
                class_name = max(semantic_counts, key=semantic_counts.get) if semantic_counts else 'occupied'
                r, g, b, a = _class_color(class_name)

                cube = Marker()
                cube.header.frame_id = self.map_frame
                cube.header.stamp = stamp
                cube.ns = 'semantic_voxels'
                cube.id = marker_id
                marker_id += 1
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = (int(key[0]) + 0.5) * self.voxel_size
                cube.pose.position.y = (int(key[1]) + 0.5) * self.voxel_size
                cube.pose.position.z = (int(key[2]) + 0.5) * self.voxel_size
                cube.pose.orientation.w = 1.0
                cube.scale.x = self.voxel_size
                cube.scale.y = self.voxel_size
                cube.scale.z = self.voxel_size
                cube.color.r = r
                cube.color.g = g
                cube.color.b = b
                cube.color.a = a
                markers.markers.append(cube)

        self.marker_pub.publish(markers)

    def _append_origin_markers(self, markers, stamp, marker_id):
        yaw = math.radians(self.origin_yaw)
        arrow = Marker()
        arrow.header.frame_id = self.map_frame
        arrow.header.stamp = stamp
        arrow.ns = 'semantic_origin'
        arrow.id = marker_id
        marker_id += 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = self.origin_x
        arrow.pose.position.y = self.origin_y
        arrow.pose.position.z = 0.04
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x = 0.32
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.35
        arrow.color.a = 1.0
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = stamp
        text.ns = 'semantic_origin_label'
        text.id = marker_id
        marker_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.origin_x
        text.pose.position.y = self.origin_y
        text.pose.position.z = 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.r = 0.0
        text.color.g = 1.0
        text.color.b = 0.35
        text.color.a = 1.0
        text.text = 'origin / 起点'
        markers.markers.append(text)
        return marker_id

    def _is_semantic_voxel(self, voxel):
        return any(name != 'occupied' for name in voxel.get('class_counts', {}))

    def _visible_voxels(self):
        semantic_voxels = []
        occupied_voxels = []
        for voxel in self.voxels:
            if self._is_semantic_voxel(voxel):
                semantic_voxels.append(voxel)
            elif int(voxel.get('count', 0)) >= self.min_occupied_count_for_marker:
                occupied_voxels.append(voxel)

        semantic_voxels.sort(key=lambda voxel: voxel.get('last_seen', 0.0), reverse=True)
        occupied_voxels.sort(
            key=lambda voxel: (
                voxel.get('count', 0),
                voxel.get('last_seen', 0.0),
            ),
            reverse=True,
        )
        semantic_count = min(len(semantic_voxels), self.max_marker_voxels)
        occupied_count = max(0, self.max_marker_voxels - semantic_count)
        return semantic_voxels[:semantic_count] + occupied_voxels[:occupied_count]


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapMarkerPublisher()
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
