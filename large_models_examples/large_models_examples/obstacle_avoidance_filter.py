#!/usr/bin/env python3
# encoding: utf-8
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def copy_twist(msg):
    twist = Twist()
    twist.linear.x = msg.linear.x
    twist.linear.y = msg.linear.y
    twist.linear.z = msg.linear.z
    twist.angular.x = msg.angular.x
    twist.angular.y = msg.angular.y
    twist.angular.z = msg.angular.z
    return twist


def normalize_angle(angle):
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    return angle


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class ObstacleAvoidanceFilter(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('enabled', True)
        self.declare_parameter('input_cmd_vel_topic', '/vllm_track/cmd_vel_raw')
        self.declare_parameter('output_cmd_vel_topic', '/controller/cmd_vel')
        self.declare_parameter('target_state_topic', '/vllm_track/target_state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.3)
        self.declare_parameter('slow_distance', 0.3)
        self.declare_parameter('scan_angle', 120.0)
        self.declare_parameter('obstacle_angle', 40.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('blocked_avoidance_timeout', 3.0)
        self.declare_parameter('target_state_timeout', 0.5)
        self.declare_parameter('target_ignore_margin', 0.12)
        self.declare_parameter('target_ignore_angle', 25.0)
        self.declare_parameter('target_lost_search_timeout', 4.0)
        self.declare_parameter('search_turn_speed', 0.18)
        self.declare_parameter('search_swing_period', 2.4)
        self.declare_parameter('avoid_release_distance', 0.36)
        self.declare_parameter('avoid_release_time', 0.6)
        self.declare_parameter('danger_distance', 0.18)
        self.declare_parameter('danger_reverse_speed', 0.03)
        self.declare_parameter('avoid_forward_speed', 0.0)
        self.declare_parameter('avoid_turn_speed', 0.12)
        self.declare_parameter('avoid_strafe_speed', 0.12)
        self.declare_parameter('tracking_angular_gain', 0.25)
        self.declare_parameter('max_avoid_angular_z', 0.3)
        self.declare_parameter('invert_avoid_direction', False)
        self.declare_parameter('max_linear_x', 0.2)
        self.declare_parameter('max_linear_y', 0.18)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('stop_on_scan_timeout', True)
        self.declare_parameter('debug', False)

        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.scan_angle = float(self.get_parameter('scan_angle').value)
        self.obstacle_angle = float(self.get_parameter('obstacle_angle').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.blocked_avoidance_timeout = float(self.get_parameter('blocked_avoidance_timeout').value)
        self.target_state_timeout = float(self.get_parameter('target_state_timeout').value)
        self.target_ignore_margin = float(self.get_parameter('target_ignore_margin').value)
        self.target_ignore_angle = math.radians(float(self.get_parameter('target_ignore_angle').value))
        self.target_lost_search_timeout = float(self.get_parameter('target_lost_search_timeout').value)
        self.search_turn_speed = float(self.get_parameter('search_turn_speed').value)
        self.search_swing_period = float(self.get_parameter('search_swing_period').value)
        self.avoid_release_distance = float(self.get_parameter('avoid_release_distance').value)
        self.avoid_release_time = float(self.get_parameter('avoid_release_time').value)
        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.danger_reverse_speed = float(self.get_parameter('danger_reverse_speed').value)
        self.avoid_forward_speed = float(self.get_parameter('avoid_forward_speed').value)
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.avoid_strafe_speed = float(self.get_parameter('avoid_strafe_speed').value)
        self.tracking_angular_gain = float(self.get_parameter('tracking_angular_gain').value)
        self.max_avoid_angular_z = float(self.get_parameter('max_avoid_angular_z').value)
        self.invert_avoid_direction = as_bool(self.get_parameter('invert_avoid_direction').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.stop_on_scan_timeout = as_bool(self.get_parameter('stop_on_scan_timeout').value)
        self.debug = as_bool(self.get_parameter('debug').value)

        if self.avoid_release_distance <= self.safety_distance:
            self.avoid_release_distance = self.safety_distance + 0.06

        input_cmd_vel_topic = self.get_parameter('input_cmd_vel_topic').value
        output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').value
        target_state_topic = self.get_parameter('target_state_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.last_cmd = Twist()
        self.last_tracking_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_forward_cmd_time = 0.0
        self.last_scan_time = 0.0
        self.target_distance = math.inf
        self.target_offset = 0.0
        self.target_visible = False
        self.last_target_time = 0.0
        self.last_target_seen_time = 0.0
        self.last_target_offset = 0.0
        self.front_min = math.inf
        self.front_min_angle = 0.0
        self.obstacle_min = math.inf
        self.obstacle_min_angle = 0.0
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_turn_dir = 1.0
        self.avoiding = False
        self.avoid_side = 1.0
        self.avoid_dir = 1.0
        self.clear_since = None
        self.blind_approaching = False
        self.target_lost_during_avoidance = False
        self.search_start_time = None
        self.search_direction = 1.0
        self.last_debug_time = 0.0

        scan_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_sub = self.create_subscription(Twist, input_cmd_vel_topic, self.cmd_callback, 10)
        self.target_sub = self.create_subscription(Vector3, target_state_topic, self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, scan_qos)
        self.cmd_pub = self.create_publisher(Twist, output_cmd_vel_topic, 1)
        self.timer = self.create_timer(0.05, self.publish_filtered_cmd)

        self.get_logger().info(
            'obstacle avoidance filter started: input=%s output=%s scan=%s target=%s trigger=%.2fm obstacle_angle=%.1fdeg scan_angle=%.1fdeg'
            % (input_cmd_vel_topic, output_cmd_vel_topic, scan_topic, target_state_topic,
               self.safety_distance, self.obstacle_angle, self.scan_angle)
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_callback(self, msg):
        self.last_cmd = copy_twist(msg)
        self.last_cmd_time = self.now_seconds()
        if msg.linear.x > 0.02:
            self.last_forward_cmd_time = self.last_cmd_time
            self.last_tracking_cmd = copy_twist(msg)

    def target_callback(self, msg):
        now = self.now_seconds()
        self.target_visible = msg.z > 0.5 and msg.x > 0.0
        self.last_target_time = now
        if self.target_visible:
            self.target_distance = msg.x
            self.target_offset = clamp(msg.y, -1.0, 1.0)
            self.last_target_seen_time = now
            self.last_target_offset = self.target_offset
            self.blind_approaching = False
            self.target_lost_during_avoidance = False
            self.search_start_time = None

    def scan_callback(self, msg):
        angle_limit = math.radians(self.scan_angle) / 2.0
        obstacle_angle_limit = math.radians(self.obstacle_angle) / 2.0
        front = []
        left = []
        right = []
        front_min = math.inf
        front_min_angle = 0.0
        obstacle_min = math.inf
        obstacle_min_angle = 0.0

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance <= 0.0:
                continue
            if msg.range_min > 0.0 and distance < msg.range_min:
                continue
            if msg.range_max > 0.0 and distance > msg.range_max:
                continue

            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if abs(angle) > angle_limit:
                continue

            front.append(distance)
            if distance < front_min:
                front_min = distance
                front_min_angle = angle
            if abs(angle) <= obstacle_angle_limit and distance < obstacle_min:
                obstacle_min = distance
                obstacle_min_angle = angle
            if angle >= 0:
                left.append(distance)
            else:
                right.append(distance)

        self.front_min = front_min if front else math.inf
        self.front_min_angle = front_min_angle
        self.obstacle_min = obstacle_min
        self.obstacle_min_angle = obstacle_min_angle
        self.left_min = min(left) if left else math.inf
        self.right_min = min(right) if right else math.inf
        self.last_scan_time = self.now_seconds()

    def publish_filtered_cmd(self):
        now = self.now_seconds()
        if now - self.last_cmd_time > self.command_timeout:
            self.cmd_pub.publish(Twist())
            return

        if not self.enabled:
            self.cmd_pub.publish(self.limit_twist(copy_twist(self.last_cmd)))
            return

        if now - self.last_scan_time > self.scan_timeout:
            if self.stop_on_scan_timeout:
                self.cmd_pub.publish(Twist())
            else:
                self.cmd_pub.publish(self.limit_twist(copy_twist(self.last_cmd)))
            return

        filtered = self.apply_avoidance(copy_twist(self.last_cmd))
        self.cmd_pub.publish(self.limit_twist(filtered))

    def apply_avoidance(self, cmd):
        now = self.now_seconds()
        cmd.linear.y = 0.0
        recently_moving_forward = now - self.last_forward_cmd_time <= self.blocked_avoidance_timeout
        obstacle_distance = self.get_obstacle_distance(now)

        if (math.isfinite(obstacle_distance)
                and obstacle_distance <= self.safety_distance
                and (cmd.linear.x > 0.02
                     or recently_moving_forward
                     or self.blind_approaching
                     or self.avoiding)):
            self.start_or_update_avoidance()

        if self.avoiding:
            self.blind_approaching = False
            if not self.is_target_current(now):
                self.target_lost_during_avoidance = True

            if not math.isfinite(obstacle_distance) or obstacle_distance >= self.avoid_release_distance:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.avoid_release_time:
                    self.avoiding = False
                    self.clear_since = None
                    if self.target_lost_during_avoidance:
                        return self.build_search_cmd(now)
                    return cmd
            else:
                self.clear_since = None
                self.maybe_switch_avoid_direction()

            return self.build_avoidance_cmd(cmd, obstacle_distance)

        if self.should_blind_approach(now, obstacle_distance):
            return self.build_blind_approach_cmd(obstacle_distance)

        self.blind_approaching = False
        if self.should_search_for_target(now, cmd):
            return self.build_search_cmd(now)

        return cmd

    def get_obstacle_distance(self, now):
        if not math.isfinite(self.obstacle_min):
            return math.inf
        if self.is_target_echo(now, self.obstacle_min, self.obstacle_min_angle):
            return math.inf
        return self.obstacle_min

    def is_target_echo(self, now, distance, angle):
        if not self.target_visible:
            return False
        if now - self.last_target_time > self.target_state_timeout:
            return False
        if not math.isfinite(self.target_distance) or self.target_distance <= 0.0:
            return False
        target_angle = self.target_offset * math.radians(self.scan_angle) / 2.0
        angle_error = min(abs(angle - target_angle), abs(angle + target_angle))
        if angle_error > self.target_ignore_angle:
            return False
        if abs(distance - self.target_distance) <= self.target_ignore_margin:
            return True
        if self.target_distance <= self.safety_distance + self.target_ignore_margin:
            return distance >= self.target_distance - self.target_ignore_margin
        return False

    def should_search_for_target(self, now, cmd):
        if self.is_target_current(now):
            return False
        if now - self.last_target_seen_time > self.target_lost_search_timeout:
            return False
        return abs(cmd.linear.x) < 0.02 and abs(cmd.angular.z) < 0.05

    def should_blind_approach(self, now, obstacle_distance):
        if self.is_target_current(now):
            return False
        if self.last_target_seen_time <= 0.0:
            return False
        if not math.isfinite(obstacle_distance):
            return False
        if obstacle_distance <= self.safety_distance:
            return False
        return self.last_tracking_cmd.linear.x > 0.02

    def is_target_current(self, now):
        return self.target_visible and now - self.last_target_time <= self.target_state_timeout

    def build_blind_approach_cmd(self, obstacle_distance):
        self.blind_approaching = True
        self.target_lost_during_avoidance = True
        self.search_start_time = None

        twist = copy_twist(self.last_tracking_cmd)
        twist.linear.y = 0.0
        if twist.linear.x < 0.0:
            twist.linear.x = 0.0
        self.debug_log('blind_approach', obstacle_distance, 0.0, twist)
        return twist

    def build_search_cmd(self, now):
        twist = Twist()
        if self.search_start_time is None:
            self.search_start_time = now
            if abs(self.last_target_offset) > 0.1:
                self.search_direction = -1.0 if self.last_target_offset > 0 else 1.0
            else:
                self.search_direction = self.last_turn_dir

        period = max(self.search_swing_period, 0.4)
        phase = (now - self.search_start_time) % period
        direction = self.search_direction if phase < period / 2.0 else -self.search_direction
        twist.angular.z = direction * self.search_turn_speed
        self.debug_log('search', self.front_min, direction, twist)
        return twist

    def start_or_update_avoidance(self):
        if self.avoiding:
            return
        now = self.now_seconds()
        self.avoiding = True
        self.avoid_side = self.choose_turn_direction()
        self.avoid_dir = self.command_direction(self.avoid_side)
        self.clear_since = None
        self.target_lost_during_avoidance = not self.is_target_current(now)
        self.search_start_time = None
        self.debug_log('enter_avoid', self.front_min, self.avoid_dir, self.last_cmd)

    def maybe_switch_avoid_direction(self):
        selected = self.left_min if self.avoid_side > 0 else self.right_min
        opposite = self.right_min if self.avoid_side > 0 else self.left_min
        if math.isfinite(selected) and math.isfinite(opposite):
            if selected < self.safety_distance * 0.75 and opposite > selected + 0.25:
                self.avoid_side *= -1.0
                self.avoid_dir = self.command_direction(self.avoid_side)
                self.last_turn_dir = self.avoid_side

    def build_avoidance_cmd(self, cmd, front_min):
        filtered = Twist()
        if front_min <= self.danger_distance:
            filtered.linear.x = -self.danger_reverse_speed
        elif front_min <= self.safety_distance:
            filtered.linear.x = 0.0
        else:
            requested_forward = max(cmd.linear.x, self.avoid_forward_speed * 0.6)
            filtered.linear.x = min(requested_forward, self.avoid_forward_speed)

        filtered.linear.y = self.avoid_dir * self.avoid_strafe_speed
        tracking_turn = clamp(
            cmd.angular.z * self.tracking_angular_gain,
            -self.max_avoid_angular_z * 0.5,
            self.max_avoid_angular_z * 0.5)
        filtered.angular.z = clamp(
            tracking_turn + self.avoid_dir * self.avoid_turn_speed,
            -self.max_avoid_angular_z,
            self.max_avoid_angular_z)
        self.debug_log('avoid', front_min, self.avoid_dir, filtered)
        return filtered

    def choose_turn_direction(self):
        if math.isfinite(self.left_min) and math.isfinite(self.right_min):
            if abs(self.left_min - self.right_min) < 0.05:
                return self.last_turn_dir
            self.last_turn_dir = -1.0 if self.left_min < self.right_min else 1.0
            return self.last_turn_dir
        if math.isfinite(self.left_min):
            self.last_turn_dir = -1.0
        elif math.isfinite(self.right_min):
            self.last_turn_dir = 1.0
        return self.last_turn_dir

    def get_avoid_direction(self):
        direction = self.choose_turn_direction()
        return self.command_direction(direction)

    def command_direction(self, direction):
        return -direction if self.invert_avoid_direction else direction

    def limit_twist(self, twist):
        twist.linear.x = clamp(twist.linear.x, -self.max_linear_x, self.max_linear_x)
        twist.linear.y = clamp(twist.linear.y, -self.max_linear_y, self.max_linear_y)
        twist.angular.z = clamp(twist.angular.z, -self.max_angular_z, self.max_angular_z)
        return twist

    def debug_log(self, state, front_min, turn_dir, cmd):
        if not self.debug:
            return
        now = self.now_seconds()
        if now - self.last_debug_time < 0.5:
            return
        self.last_debug_time = now
        self.get_logger().info(
            '%s obstacle=%.2f wide_front=%.2f left=%.2f right=%.2f turn=%+.0f cmd=(%.2f, %.2f, %.2f)'
            % (state, front_min, self.front_min, self.left_min, self.right_min, turn_dir,
               cmd.linear.x, cmd.linear.y, cmd.angular.z)
        )


def main():
    rclpy.init()
    node = ObstacleAvoidanceFilter('obstacle_avoidance_filter')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
