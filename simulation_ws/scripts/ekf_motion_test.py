#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EkfMotionTest(Node):
    def __init__(self):
        super().__init__(
            'ekf_motion_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.raw = None
        self.filtered = None
        self.truth = None
        self.create_subscription(Odometry, '/odom', self._raw_cb, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self._filtered_cb, 20)
        self.create_subscription(
            TFMessage, '/world/follower_world/dynamic_pose/info',
            self._truth_cb, 20)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def _raw_cb(self, msg):
        self.raw = msg

    def _filtered_cb(self, msg):
        self.filtered = msg

    def _truth_cb(self, msg):
        for transform in msg.transforms:
            # ros_gz_bridge currently drops Pose_V names when converting to
            # TFMessage. The trolley is the only dynamic root pose in this
            # world whose global height stays in this interval.
            z = transform.transform.translation.z
            if 0.35 < z < 0.60:
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                self.truth = pose
                return

    def spin_for_sim_time(self, seconds, command=None, wall_timeout=30.0):
        start_sim = self.get_clock().now().nanoseconds
        end_sim = start_sim + int(seconds * 1e9)
        end_wall = time.monotonic() + wall_timeout
        while self.get_clock().now().nanoseconds < end_sim:
            if time.monotonic() >= end_wall:
                raise TimeoutError('El reloj simulado no avanzo a tiempo')
            if command is not None:
                self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_inputs(self, wall_timeout=15.0):
        end_wall = time.monotonic() + wall_timeout
        stop = Twist()
        while self.raw is None or self.filtered is None or self.truth is None:
            if time.monotonic() >= end_wall:
                missing = []
                if self.raw is None:
                    missing.append('/odom')
                if self.filtered is None:
                    missing.append('/odometry/filtered')
                if self.truth is None:
                    missing.append('/world/follower_world/dynamic_pose/info:trolley')
                raise TimeoutError('Sin datos de: ' + ', '.join(missing))
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.10)


def pose_dict(msg):
    pose = msg if isinstance(msg, Pose) else msg.pose.pose
    return {'x': pose.position.x, 'y': pose.position.y,
            'yaw_rad': yaw_from_quaternion(pose.orientation)}


def main():
    rclpy.init()
    node = EkfMotionTest()
    try:
        node.wait_for_inputs()
        node.spin_for_sim_time(0.5, Twist())
        start_raw = pose_dict(node.raw)
        start_filtered = pose_dict(node.filtered)
        start_truth = pose_dict(node.truth)

        forward = Twist()
        forward.linear.x = 0.20
        wall_start = time.monotonic()
        node.spin_for_sim_time(2.0, forward)
        node.spin_for_sim_time(1.0, Twist())
        wall_elapsed = time.monotonic() - wall_start

        end_raw = pose_dict(node.raw)
        end_filtered = pose_dict(node.filtered)
        end_truth = pose_dict(node.truth)

        turn_start_raw = pose_dict(node.raw)
        turn_start_filtered = pose_dict(node.filtered)
        turn_start_truth = pose_dict(node.truth)
        turn = Twist()
        turn.angular.z = 0.30
        turn_wall_start = time.monotonic()
        node.spin_for_sim_time(2.0, turn)
        node.spin_for_sim_time(1.0, Twist())
        turn_wall_elapsed = time.monotonic() - turn_wall_start
        turn_end_raw = pose_dict(node.raw)
        turn_end_filtered = pose_dict(node.filtered)
        turn_end_truth = pose_dict(node.truth)

        result = {
            'command': {'linear_m_s': 0.20, 'duration_s': 2.0,
                        'expected_distance_m_with_acceleration_ramp': 0.36},
            'raw_delta_m': end_raw['x'] - start_raw['x'],
            'filtered_delta_m': end_filtered['x'] - start_filtered['x'],
            'ground_truth_delta_m': end_truth['x'] - start_truth['x'],
            'raw_lateral_m': end_raw['y'] - start_raw['y'],
            'filtered_lateral_m': end_filtered['y'] - start_filtered['y'],
            'raw_yaw_delta_rad': end_raw['yaw_rad'] - start_raw['yaw_rad'],
            'filtered_yaw_delta_rad': end_filtered['yaw_rad'] - start_filtered['yaw_rad'],
            'raw_filtered_x_difference_m': end_filtered['x'] - end_raw['x'],
            'wall_time_for_3_sim_seconds_s': wall_elapsed,
            'real_time_factor_estimate': 3.0 / wall_elapsed,
            'turn_command': {
                'angular_rad_s': 0.30,
                'duration_s': 2.0,
                'expected_yaw_rad_with_acceleration_ramp': 0.60,
            },
            'turn_raw_yaw_delta_rad': (
                turn_end_raw['yaw_rad'] - turn_start_raw['yaw_rad']),
            'turn_filtered_yaw_delta_rad': (
                turn_end_filtered['yaw_rad'] - turn_start_filtered['yaw_rad']),
            'turn_ground_truth_yaw_delta_rad': (
                turn_end_truth['yaw_rad'] - turn_start_truth['yaw_rad']),
            'turn_raw_filtered_difference_rad': (
                turn_end_filtered['yaw_rad'] - turn_end_raw['yaw_rad']),
            'turn_wall_time_for_3_sim_seconds_s': turn_wall_elapsed,
            'turn_real_time_factor_estimate': 3.0 / turn_wall_elapsed,
        }
        print(json.dumps(result, indent=2))
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
