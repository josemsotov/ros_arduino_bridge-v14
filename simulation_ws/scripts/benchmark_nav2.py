#!/usr/bin/env python3
"""Reproducible headless Nav2 benchmark using wall and simulation clocks."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class NavigationBenchmark(Node):
    def __init__(self, x: float, y: float, yaw: float) -> None:
        super().__init__('navigation_benchmark')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.sim_time = None
        self.amcl_pose = None
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw

    def _clock_cb(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg.pose.pose

    def run(self, timeout_s: float) -> dict:
        if not self.client.wait_for_server(timeout_sec=60.0):
            return {'status': 'SERVER_UNAVAILABLE'}

        # Ensure both clocks are available before starting the timed section.
        clock_deadline = time.monotonic() + 90.0
        while self.sim_time is None and time.monotonic() < clock_deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.sim_time is None:
            return {'status': 'SIM_CLOCK_UNAVAILABLE'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        wall_start = time.monotonic()
        sim_start = self.sim_time
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {'status': 'REJECTED'}

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        wall_end = time.monotonic()
        sim_end = self.sim_time

        if not result_future.done():
            handle.cancel_goal_async()
            status = 'TIMEOUT'
            action_status = None
        else:
            wrapped = result_future.result()
            action_status = int(wrapped.status)
            status = 'SUCCEEDED' if action_status == 4 else 'FAILED'

        output = {
            'status': status,
            'action_status_code': action_status,
            'goal': {'x_m': self.goal_x, 'y_m': self.goal_y,
                     'yaw_rad': self.goal_yaw},
            'wall_duration_s': round(wall_end - wall_start, 3),
            'sim_start_s': sim_start,
            'sim_end_s': sim_end,
        }
        if sim_start is not None and sim_end is not None:
            sim_duration = sim_end - sim_start
            output['sim_duration_s'] = round(sim_duration, 3)
            output['real_time_factor'] = round(
                sim_duration / (wall_end - wall_start), 4)
        if self.amcl_pose is not None:
            pose = self.amcl_pose
            output['amcl_final'] = {
                'x_m': pose.position.x,
                'y_m': pose.position.y,
                'yaw_rad': 2.0 * math.atan2(
                    pose.orientation.z, pose.orientation.w),
            }
        return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=1.0)
    parser.add_argument('--y', type=float, default=-1.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=600.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavigationBenchmark(args.x, args.y, args.yaw)
    try:
        result = node.run(args.timeout)
        print('BENCHMARK_JSON=' + json.dumps(result, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
