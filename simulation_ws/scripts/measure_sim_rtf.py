#!/usr/bin/env python3
"""Measure Gazebo real-time factor without commanding the robot."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockMonitor(Node):
    def __init__(self) -> None:
        super().__init__('simulation_clock_monitor')
        self.sim_time = None
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)

    def _clock_cb(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = ClockMonitor()
    deadline = time.monotonic() + 30.0
    while node.sim_time is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.sim_time is None:
        raise RuntimeError('/clock is not publishing')

    sim_start = node.sim_time
    wall_start = time.monotonic()
    while time.monotonic() - wall_start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall_duration = time.monotonic() - wall_start
    sim_duration = node.sim_time - sim_start
    print(json.dumps({
        'wall_duration_s': round(wall_duration, 3),
        'sim_duration_s': round(sim_duration, 3),
        'real_time_factor': round(sim_duration / wall_duration, 4),
    }, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
