#!/usr/bin/env python3
# encoding: utf-8

import json
import math
import os
import time
from collections import defaultdict

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from interfaces.msg import ObjectsInfo
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _now_sec(node):
    return node.get_clock().now().nanoseconds / 1e9


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


def _quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SemanticVoxelMapper(Node):
    """Build a sparse semantic voxel map from YOLO detections and aligned depth."""

    def __init__(self):
        super().__init__('semantic_voxel_mapper')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('depth_topic', '/depth_cam/depth0/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_cam/rgb0/camera_info')
        self.declare_parameter('objects_topic', '/yolo/object_detect')
        self.declare_parameter('map_file', '~/.ros/semantic_voxel_map.json')
        self.declare_parameter('load_existing_map', False)
        self.declare_parameter('voxel_size', 0.12)
        self.declare_parameter('sample_stride', 8)
        self.declare_parameter('center_crop_ratio', 0.65)
        self.declare_parameter('object_depth_percentile', 35.0)
        self.declare_parameter('object_depth_band', 0.35)
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('min_score', 0.35)
        self.declare_parameter('merge_distance', 0.45)
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('max_saved_voxels', 12000)
        self.declare_parameter('publish_voxel_markers', False)
        self.declare_parameter('integrate_depth_map', False)
        self.declare_parameter('depth_map_stride', 36)
        self.declare_parameter('depth_map_period', 2.0)
        self.declare_parameter('max_depth_points_per_update', 700)
        self.declare_parameter('max_occupied_voxels', 6000)
        self.declare_parameter('max_marker_voxels', 700)
        self.declare_parameter('min_occupied_count_for_marker', 2)
        self.declare_parameter('min_occupied_count_for_save', 2)

        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame_param = self.get_parameter('camera_frame').value
        self.map_file = _expand_path(self.get_parameter('map_file').value)
        self.load_existing_map = _as_bool(self.get_parameter('load_existing_map').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.sample_stride = max(1, int(self.get_parameter('sample_stride').value))
        self.center_crop_ratio = float(self.get_parameter('center_crop_ratio').value)
        self.object_depth_percentile = float(self.get_parameter('object_depth_percentile').value)
        self.object_depth_band = max(0.02, float(self.get_parameter('object_depth_band').value))
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.merge_distance = float(self.get_parameter('merge_distance').value)
        self.max_saved_voxels = int(self.get_parameter('max_saved_voxels').value)
        self.publish_voxel_markers = _as_bool(self.get_parameter('publish_voxel_markers').value)
        self.integrate_depth_map = _as_bool(self.get_parameter('integrate_depth_map').value)
        self.depth_map_stride = max(1, int(self.get_parameter('depth_map_stride').value))
        self.depth_map_period = float(self.get_parameter('depth_map_period').value)
        self.max_depth_points_per_update = max(1, int(self.get_parameter('max_depth_points_per_update').value))
        self.max_occupied_voxels = max(0, int(self.get_parameter('max_occupied_voxels').value))
        self.max_marker_voxels = int(self.get_parameter('max_marker_voxels').value)
        self.min_occupied_count_for_marker = max(1, int(self.get_parameter('min_occupied_count_for_marker').value))
        self.min_occupied_count_for_save = max(1, int(self.get_parameter('min_occupied_count_for_save').value))
        self.publish_period = max(0.1, float(self.get_parameter('publish_period').value))
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.objects_topic = self.get_parameter('objects_topic').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_depth = None
        self.latest_depth_frame = ''
        self.latest_camera_info = None

        self.voxels = {}
        self.objects = {}
        self.class_next_id = defaultdict(int)
        self.last_depth_map_update = 0.0
        self.object_msg_count = 0
        self.detected_object_count = 0
        self.projected_object_count = 0
        self.object_point_count = 0
        self.depth_point_count = 0
        self.last_empty_projection_log = 0.0

        if self.load_existing_map:
            self._load_existing_map()
        else:
            self.get_logger().info('Starting a fresh semantic map; existing map file will not be loaded')

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            1,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            1,
        )
        self.create_subscription(
            ObjectsInfo,
            self.objects_topic,
            self.objects_callback,
            1,
        )

        self.summary_pub = self.create_publisher(String, '/semantic_map/objects', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map/markers', 1)
        self.create_service(Trigger, '~/save', self.save_callback)
        self.last_status_log = 0.0
        self.create_timer(self.publish_period, self.periodic_publish)

        self.get_logger().info(
            f'Semantic voxel mapper started, manual save service: ~/save -> {self.map_file}'
        )
        self.get_logger().info(
            'Semantic voxel mapper inputs: depth=%s camera_info=%s objects=%s '
            'publish_voxel_markers=%s integrate_depth_map=%s depth_stride=%d max_depth_points=%d '
            'max_occupied_voxels=%d object_depth_percentile=%.1f object_depth_band=%.2f'
            % (
                self.depth_topic,
                self.camera_info_topic,
                self.objects_topic,
                self.publish_voxel_markers,
                self.integrate_depth_map,
                self.depth_map_stride,
                self.max_depth_points_per_update,
                self.max_occupied_voxels,
                self.object_depth_percentile,
                self.object_depth_band,
            )
        )

    def _load_existing_map(self):
        if not os.path.exists(self.map_file):
            return
        try:
            with open(self.map_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            for obj in data.get('objects', []):
                object_id = obj.get('id')
                if object_id:
                    self.objects[object_id] = obj
                    class_name = obj.get('class_name', 'object')
                    suffix = object_id.rsplit('_', 1)[-1]
                    if suffix.isdigit():
                        self.class_next_id[class_name] = max(self.class_next_id[class_name], int(suffix))
            for voxel in data.get('voxels', []):
                key = tuple(voxel.get('key', []))
                if len(key) == 3:
                    self.voxels[key] = {
                        'count': int(voxel.get('count', 0)),
                        'class_counts': dict(voxel.get('class_counts', {})),
                        'last_seen': float(voxel.get('last_seen', 0.0)),
                    }
            self.get_logger().info(
                f'Loaded {len(self.objects)} objects and {len(self.voxels)} voxels from existing map'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to load semantic map {self.map_file}: {e}')

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = np.array(depth)
            self.latest_depth_frame = msg.header.frame_id
            if self.integrate_depth_map:
                self._integrate_depth_map(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode depth image: {e}', throttle_duration_sec=2.0)

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def objects_callback(self, msg):
        self.object_msg_count += 1
        self.detected_object_count += len(msg.objects)
        if self.latest_depth is None or self.latest_camera_info is None:
            self.get_logger().warn('Waiting for depth image and camera info', throttle_duration_sec=2.0)
            return

        camera_frame = (
            self.camera_frame_param
            or self.latest_depth_frame
            or self.latest_camera_info.header.frame_id
        )
        if not camera_frame:
            self.get_logger().warn('No camera frame available for semantic projection', throttle_duration_sec=2.0)
            return

        transform = self._lookup_transform(camera_frame)
        if transform is None:
            return

        for obj in msg.objects:
            if hasattr(obj, 'score') and obj.score < self.min_score:
                continue
            points = self._points_from_detection(obj, self.latest_depth, self.latest_camera_info)
            if points.size == 0:
                now = _now_sec(self)
                if now - self.last_empty_projection_log > 2.0:
                    self.last_empty_projection_log = now
                    self.get_logger().warn(
                        'YOLO object has no valid depth samples: class=%s score=%.2f box=%s depth_shape=%s'
                        % (
                            getattr(obj, 'class_name', 'object'),
                            float(getattr(obj, 'score', 0.0)),
                            list(getattr(obj, 'box', [])),
                            tuple(self.latest_depth.shape),
                        )
                    )
                continue

            map_points = self._transform_points(points, transform)
            if map_points.size == 0:
                continue

            now = _now_sec(self)
            centroid = np.median(map_points, axis=0)
            self._update_voxels(obj.class_name, map_points, now)
            self._merge_object_observation(obj.class_name, centroid, float(obj.score), len(map_points), now)
            self.projected_object_count += 1
            self.object_point_count += int(map_points.shape[0])

    def _lookup_transform(self, camera_frame):
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, camera_frame, Time())
        except Exception as e:
            self.get_logger().warn(
                f'Cannot transform {camera_frame} to {self.map_frame}: {e}',
                throttle_duration_sec=2.0,
            )
            return None

    def _integrate_depth_map(self, msg):
        now = _now_sec(self)
        if now - self.last_depth_map_update < self.depth_map_period:
            return
        if self.latest_camera_info is None:
            return

        camera_frame = (
            self.camera_frame_param
            or msg.header.frame_id
            or self.latest_camera_info.header.frame_id
        )
        if not camera_frame:
            return

        transform = self._lookup_transform(camera_frame)
        if transform is None:
            return

        points = self._points_from_depth_image(self.latest_depth, self.latest_camera_info, self.depth_map_stride)
        if points.size == 0:
            return

        map_points = self._transform_points(points, transform)
        if map_points.size == 0:
            return
        if map_points.shape[0] > self.max_depth_points_per_update:
            idx = np.linspace(
                0,
                map_points.shape[0] - 1,
                self.max_depth_points_per_update,
                dtype=np.int32,
            )
            map_points = map_points[idx]

        self.last_depth_map_update = now
        self._update_voxels('occupied', map_points, now)
        self.depth_point_count += int(map_points.shape[0])
        self._prune_occupied_voxels()

    def _points_from_depth_image(self, depth_image, camera_info, stride):
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        camera_cx = camera_info.k[2]
        camera_cy = camera_info.k[5]
        if fx == 0.0 or fy == 0.0:
            return np.empty((0, 3), dtype=np.float32)

        us = np.arange(0, depth_image.shape[1], stride, dtype=np.int32)
        vs = np.arange(0, depth_image.shape[0], stride, dtype=np.int32)
        if us.size == 0 or vs.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        uu, vv = np.meshgrid(us, vs)
        z = depth_image[vv, uu].astype(np.float32)
        if np.issubdtype(depth_image.dtype, np.integer) or np.nanmax(z) > 20.0:
            z = z / 1000.0

        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid]

        x = (uu - camera_cx) * z / fx
        y = (vv - camera_cy) * z / fy
        return np.stack([x, y, z], axis=1)

    def _filter_object_depth_band(self, z):
        if z.size == 0:
            return z, np.zeros((0,), dtype=bool)
        percentile = min(95.0, max(5.0, self.object_depth_percentile))
        surface_depth = float(np.percentile(z, percentile))
        mask = np.abs(z - surface_depth) <= self.object_depth_band
        if not np.any(mask):
            mask = np.ones(z.shape, dtype=bool)
        return z[mask], mask

    def _points_from_detection(self, obj, depth_image, camera_info):
        x1, y1, x2, y2 = self._box_to_xyxy(obj.box, obj.width, obj.height)
        if x2 <= x1 or y2 <= y1:
            return np.empty((0, 3), dtype=np.float32)

        width = x2 - x1
        height = y2 - y1
        crop = max(0.1, min(1.0, self.center_crop_ratio))
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        x1 = int(max(0, cx - width * crop / 2.0))
        x2 = int(min(depth_image.shape[1] - 1, cx + width * crop / 2.0))
        y1 = int(max(0, cy - height * crop / 2.0))
        y2 = int(min(depth_image.shape[0] - 1, cy + height * crop / 2.0))

        if x2 <= x1 or y2 <= y1:
            return np.empty((0, 3), dtype=np.float32)

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        camera_cx = camera_info.k[2]
        camera_cy = camera_info.k[5]
        if fx == 0.0 or fy == 0.0:
            return np.empty((0, 3), dtype=np.float32)

        us = np.arange(x1, x2, self.sample_stride, dtype=np.int32)
        vs = np.arange(y1, y2, self.sample_stride, dtype=np.int32)
        if us.size == 0 or vs.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        uu, vv = np.meshgrid(us, vs)
        z = depth_image[vv, uu].astype(np.float32)
        if np.issubdtype(depth_image.dtype, np.integer) or np.nanmax(z) > 20.0:
            z = z / 1000.0

        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid]
        z, foreground_mask = self._filter_object_depth_band(z)
        if z.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        uu = uu[foreground_mask]
        vv = vv[foreground_mask]

        x = (uu - camera_cx) * z / fx
        y = (vv - camera_cy) * z / fy
        return np.stack([x, y, z], axis=1)

    def _box_to_xyxy(self, box, image_width, image_height):
        if len(box) < 4:
            return 0, 0, 0, 0
        a, b, c, d = [int(v) for v in box[:4]]
        if c > a and d > b:
            x1, y1, x2, y2 = a, b, c, d
        else:
            x1 = int(a - c / 2.0)
            y1 = int(b - d / 2.0)
            x2 = int(a + c / 2.0)
            y2 = int(b + d / 2.0)
        max_x = image_width - 1 if image_width else 10**9
        max_y = image_height - 1 if image_height else 10**9
        return max(0, x1), max(0, y1), min(max_x, x2), min(max_y, y2)

    def _transform_points(self, points, transform):
        rot = _quat_to_matrix(transform.transform.rotation)
        trans = transform.transform.translation
        t = np.array([trans.x, trans.y, trans.z], dtype=np.float32)
        return points @ rot.T + t

    def _update_voxels(self, class_name, map_points, now):
        keys = np.floor(map_points / self.voxel_size).astype(np.int32)
        for key_arr in keys:
            key = tuple(int(v) for v in key_arr)
            voxel = self.voxels.setdefault(key, {
                'count': 0,
                'class_counts': {},
                'last_seen': now,
            })
            voxel['count'] += 1
            voxel['class_counts'][class_name] = voxel['class_counts'].get(class_name, 0) + 1
            voxel['last_seen'] = now

    def _is_semantic_voxel(self, voxel):
        return any(name != 'occupied' for name in voxel.get('class_counts', {}))

    def _prune_occupied_voxels(self):
        if self.max_occupied_voxels <= 0:
            return
        occupied_items = [
            (key, voxel)
            for key, voxel in self.voxels.items()
            if not self._is_semantic_voxel(voxel)
        ]
        if len(occupied_items) <= int(self.max_occupied_voxels * 1.15):
            return
        occupied_items.sort(
            key=lambda item: (
                item[1].get('count', 0),
                item[1].get('last_seen', 0.0),
            ),
            reverse=True,
        )
        keep = {key for key, _ in occupied_items[:self.max_occupied_voxels]}
        for key, _ in occupied_items[self.max_occupied_voxels:]:
            if key not in keep:
                self.voxels.pop(key, None)

    def _merge_object_observation(self, class_name, centroid, score, point_count, now):
        best_id = None
        best_dist = float('inf')
        for object_id, obj in self.objects.items():
            if obj.get('class_name') != class_name:
                continue
            pos = np.array(obj.get('position', [0.0, 0.0, 0.0]), dtype=np.float32)
            dist = float(np.linalg.norm(pos[:2] - centroid[:2]))
            if dist < best_dist:
                best_dist = dist
                best_id = object_id

        if best_id is None or best_dist > self.merge_distance:
            self.class_next_id[class_name] += 1
            best_id = f'{class_name.replace(" ", "_")}_{self.class_next_id[class_name]}'
            self.objects[best_id] = {
                'id': best_id,
                'class_name': class_name,
                'position': centroid.tolist(),
                'confidence': score,
                'observations': 0,
                'first_seen': now,
                'last_seen': now,
                'source': 'yolo_depth',
                'point_count': 0,
            }

        obj = self.objects[best_id]
        observations = int(obj.get('observations', 0))
        old_pos = np.array(obj.get('position', centroid.tolist()), dtype=np.float32)
        new_pos = ((old_pos * observations) + centroid) / float(observations + 1)
        obj['position'] = new_pos.tolist()
        obj['confidence'] = max(float(obj.get('confidence', 0.0)), score)
        obj['observations'] = observations + 1
        obj['last_seen'] = now
        obj['point_count'] = int(obj.get('point_count', 0)) + int(point_count)

    def save_callback(self, request, response):
        try:
            saved_voxels = self._save_map()
        except Exception as e:
            response.success = False
            response.message = f'Failed to save semantic map: {e}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'Saved {len(self.objects)} objects and '
            f'{saved_voxels} voxels to {self.map_file}'
        )
        self.get_logger().info(response.message)
        return response

    def periodic_publish(self):
        self._publish_summary()
        self._publish_markers()
        now = _now_sec(self)
        if now - self.last_status_log > 5.0:
            self.last_status_log = now
            semantic_voxels = self._semantic_voxel_count()
            self.get_logger().info(
                'Live semantic map: objects=%d voxels=%d semantic_voxels=%d '
                'object_msgs=%d detections=%d projected=%d object_points=%d depth_points=%d '
                'depth=%s camera_info=%s frame=%s'
                % (
                    len(self.objects),
                    len(self.voxels),
                    semantic_voxels,
                    self.object_msg_count,
                    self.detected_object_count,
                    self.projected_object_count,
                    self.object_point_count,
                    self.depth_point_count,
                    'ok' if self.latest_depth is not None else 'waiting',
                    'ok' if self.latest_camera_info is not None else 'waiting',
                    self.map_frame,
                )
            )

    def _semantic_voxel_count(self):
        count = 0
        for voxel in self.voxels.values():
            class_counts = voxel.get('class_counts', {})
            if any(name != 'occupied' for name in class_counts):
                count += 1
        return count

    def _publish_summary(self):
        data = {
            'frame_id': self.map_frame,
            'updated_at': time.time(),
            'object_count': len(self.objects),
            'objects': list(self.objects.values()),
        }
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.summary_pub.publish(msg)

    def _publish_markers(self):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        marker_id = 1
        for obj in self.objects.values():
            pos = obj.get('position', [0.0, 0.0, 0.0])

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
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
            text.header.stamp = sphere.header.stamp
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
            visible_voxels = self._visible_voxel_items()
            for key, voxel in visible_voxels:
                class_counts = voxel.get('class_counts', {})
                semantic_counts = {
                    name: count for name, count in class_counts.items()
                    if name != 'occupied'
                }
                if semantic_counts:
                    class_name = max(semantic_counts, key=semantic_counts.get)
                else:
                    class_name = 'occupied'
                r, g, b, a = _class_color(class_name)

                cube = Marker()
                cube.header.frame_id = self.map_frame
                cube.header.stamp = self.get_clock().now().to_msg()
                cube.ns = 'semantic_voxels'
                cube.id = marker_id
                marker_id += 1
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = (key[0] + 0.5) * self.voxel_size
                cube.pose.position.y = (key[1] + 0.5) * self.voxel_size
                cube.pose.position.z = (key[2] + 0.5) * self.voxel_size
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

    def _visible_voxel_items(self):
        semantic_items = []
        occupied_items = []
        for key, voxel in self.voxels.items():
            if self._is_semantic_voxel(voxel):
                semantic_items.append((key, voxel))
            elif int(voxel.get('count', 0)) >= self.min_occupied_count_for_marker:
                occupied_items.append((key, voxel))

        semantic_items.sort(key=lambda item: item[1].get('last_seen', 0.0), reverse=True)
        occupied_items.sort(
            key=lambda item: (
                item[1].get('count', 0),
                item[1].get('last_seen', 0.0),
            ),
            reverse=True,
        )
        semantic_count = min(len(semantic_items), self.max_marker_voxels)
        occupied_count = max(0, self.max_marker_voxels - semantic_count)
        return semantic_items[:semantic_count] + occupied_items[:occupied_count]

    def _saved_voxel_items(self):
        semantic_items = []
        occupied_items = []
        for key, voxel in self.voxels.items():
            if self._is_semantic_voxel(voxel):
                semantic_items.append((key, voxel))
            elif int(voxel.get('count', 0)) >= self.min_occupied_count_for_save:
                occupied_items.append((key, voxel))

        semantic_items.sort(key=lambda item: item[1].get('last_seen', 0.0), reverse=True)
        occupied_items.sort(
            key=lambda item: (
                item[1].get('count', 0),
                item[1].get('last_seen', 0.0),
            ),
            reverse=True,
        )
        semantic_count = min(len(semantic_items), self.max_saved_voxels)
        occupied_count = max(0, self.max_saved_voxels - semantic_count)
        return semantic_items[:semantic_count] + occupied_items[:occupied_count]

    def _save_map(self):
        map_dir = os.path.dirname(self.map_file)
        if map_dir:
            os.makedirs(map_dir, exist_ok=True)
        sorted_voxels = self._saved_voxel_items()
        data = {
            'version': 1,
            'frame_id': self.map_frame,
            'voxel_size': self.voxel_size,
            'updated_at': time.time(),
            'objects': list(self.objects.values()),
            'voxels': [
                {
                    'key': list(key),
                    'count': voxel.get('count', 0),
                    'class_counts': voxel.get('class_counts', {}),
                    'last_seen': voxel.get('last_seen', 0.0),
                }
                for key, voxel in sorted_voxels
            ],
        }
        tmp_file = self.map_file + '.tmp'
        with open(tmp_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp_file, self.map_file)
        return len(sorted_voxels)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticVoxelMapper()
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
