#!/usr/bin/env python3
"""Read-only startup and throughput check; this tool never publishes commands."""

import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    def __init__(self):
        super().__init__('leap_sim_check')
        self.counts = {'clock': 0, 'scan': 0, 'odom': 0}
        self.last = {}
        self.sim_time = None
        self.last_scan = None
        self.last_odom = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_kept = [
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data),
        ]

    def seen(self, key):
        self.counts[key] += 1
        self.last[key] = time.monotonic()

    def clock(self, msg):
        self.seen('clock')
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def scan(self, msg):
        self.seen('scan')
        self.last_scan = msg

    def odom(self, msg):
        self.seen('odom')
        self.last_odom = msg

    def has_tf(self):
        return bool(self.buffer.can_transform('odom', 'base_scan', rclpy.time.Time()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 60:
        parser.error('--seconds must be 5..60 wall seconds')
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')

    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(node.counts.values()) and node.has_tf():
                break
        ready = all(node.counts.values()) and node.has_tf()
        start = time.monotonic()
        start_counts = node.counts.copy()
        start_sim = node.sim_time
        if ready:
            while time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - start
        now = time.monotonic()
        issues = [] if ready else ['startup_inputs_or_tf_missing']
        for key in node.counts:
            if now - node.last.get(key, -math.inf) > 1.0:
                issues.append(key + '_stale_or_missing')
        scan = node.last_scan
        if scan is None or not any(math.isfinite(x) and scan.range_min <= x <= scan.range_max for x in scan.ranges):
            issues.append('no_finite_scan_returns')
        odom = node.last_odom
        if odom is None or not all(math.isfinite(x) for x in (
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)):
            issues.append('odom_invalid')
        tf_ok = node.has_tf()
        if not tf_ok:
            issues.append('odom_to_scan_tf_missing')
        rtf = ((node.sim_time - start_sim) / elapsed
               if ready and elapsed > 0 and start_sim is not None else None)
        if rtf is not None and rtf <= 0:
            issues.append('clock_not_advancing')
        report = {
            'ok': not issues, 'issues': issues, 'ros_domain_id': 178,
            'robot': 'upstream TurtleBot3 Burger, not LEAP dynamics',
            'wall_seconds': elapsed, 'simulation_seconds_per_wall_second': rtf,
            'receive_hz_wall': {k: (node.counts[k] - start_counts[k]) / elapsed
                                if ready and elapsed > 0 else None for k in node.counts},
            'counts_including_startup': node.counts,
            'odom_to_base_scan_tf': tf_ok,
            'scan_frame': scan.header.frame_id if scan else None,
            'scan_rays': len(scan.ranges) if scan else None,
            'limits': 'Startup/receive check only; navigation, collisions and LEAP migration untested.',
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report), flush=True)
        return 0 if report['ok'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
