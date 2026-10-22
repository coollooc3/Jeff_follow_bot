"""Shared read-only navigation recording and pose construction."""
import csv
import json
import math
import time
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener
from expert_recorder import ExpertRecorder


def pose(node, x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


class Recorder:
    def __init__(self, node, directory, task, record_expert=False):
        self.node = node
        self.events = (directory / 'events.jsonl').open('w', encoding='utf-8')
        self.trajectory_file = (directory / 'trajectory.csv').open('w', newline='', encoding='utf-8')
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow(['wall_monotonic_s', 'sim_s', 'odom_x_m', 'odom_y_m',
                                  'map_x_m', 'map_y_m', 'map_yaw_rad',
                                  'odom_linear_mps', 'odom_angular_rps',
                                  'cmd_linear_mps', 'cmd_angular_rps', 'cmd_receive_age_wall_s'])
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.expert = ExpertRecorder(node, self.buffer, directory, task['goal']) if record_expert else None
        self.first_odom_pose = None
        self.last_odom = None
        self.odom_received = None
        self.last_cmd = None
        self.cmd_received = None
        self.nonzero_commands = 0
        self.plan_count = 0
        self.first_plan = None
        self.latest_plan = None
        self.latest_plan_wall = None
        self.path_length = 0.0
        self.previous_xy = None
        self.last_saved = 0.0
        self.stop_probe = None
        self.planner_aborted_goals = set()
        self.subscriptions = [
            node.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data),
            node.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos_profile_sensor_data),
            node.create_subscription(NavPath, '/plan', self.on_plan, qos_profile_sensor_data),
            node.create_subscription(PointStamped, '/lookahead_point', self.on_lookahead,
                                     qos_profile_sensor_data),
            node.create_subscription(NavPath, '/lookahead_collision_arc', self.on_projection,
                                     qos_profile_sensor_data),
            node.create_subscription(GoalStatusArray, '/compute_path_to_pose/_action/status',
                                     self.on_planner_status, 10),
        ]

    def event(self, state, **fields):
        self.events.write(json.dumps({'wall_monotonic_s': time.monotonic(),
                                      'state': state, **fields}) + '\n')
        self.events.flush()
        print(state, flush=True)

    def map_pose(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p, q = tf.transform.translation, tf.transform.rotation
            return [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z))]
        except TransformException:
            return None

    def on_odom(self, msg):
        self.last_odom, self.odom_received = msg, time.monotonic()
        p = msg.pose.pose.position
        if self.first_odom_pose is None:
            q = msg.pose.pose.orientation
            self.first_odom_pose = [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                       1 - 2 * (q.y * q.y + q.z * q.z))]
        if self.expert is not None:
            self.expert.on_odom(msg)
        if self.stop_probe is not None:
            self.stop_probe.observe(self.odom_received, (p.x, p.y),
                                    msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self.previous_xy is not None:
            self.path_length += math.hypot(p.x - self.previous_xy[0], p.y - self.previous_xy[1])
        self.previous_xy = (p.x, p.y)
        if self.odom_received - self.last_saved >= 0.1:
            self.last_saved = self.odom_received
            stamp = msg.header.stamp
            command = self.last_cmd
            self.trajectory.writerow([self.odom_received, stamp.sec + stamp.nanosec * 1e-9,
                                      p.x, p.y, *(self.map_pose() or ['', '', '']),
                                      msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                                      command.linear.x if command else None,
                                      command.angular.z if command else None,
                                      self.odom_received - self.cmd_received if command else None])

    def on_cmd(self, msg):
        self.last_cmd, self.cmd_received = msg, time.monotonic()
        self.nonzero_commands += int(abs(msg.linear.x) + abs(msg.angular.z) > 1e-4)
        if self.expert is not None:
            self.expert.on_executed(msg)

    def moving_forward(self, now):
        return (self.last_cmd is not None and self.last_odom is not None and
                now - self.cmd_received < 0.3 and now - self.odom_received < 0.3 and
                self.last_cmd.linear.x > 0.02 and self.last_odom.twist.twist.linear.x > 0.02)

    def on_planner_status(self, msg):
        for status in msg.status_list:
            identity = bytes(status.goal_info.goal_id.uuid).hex()
            if status.status == GoalStatus.STATUS_ABORTED and identity not in self.planner_aborted_goals:
                self.planner_aborted_goals.add(identity)
                self.event('planner_goal_aborted', goal_id=identity)

    def on_plan(self, msg):
        if self.expert is not None:
            self.expert.on_plan(msg)
        self.plan_count += 1
        self.latest_plan = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.latest_plan_wall = time.monotonic()
        if self.first_plan is None:
            self.first_plan = self.latest_plan
        # Save planner output separately from the executed odometry trajectory.
        self.events.write(json.dumps({'state': 'plan', 'frame': msg.header.frame_id,
                                      'wall_monotonic_s': self.latest_plan_wall,
                                      'poses': self.latest_plan}) + '\n')

    def on_lookahead(self, msg):
        self.events.write(json.dumps({'state': 'rpp_lookahead',
            'wall_monotonic_s': time.monotonic(), 'frame': msg.header.frame_id,
            'point': [msg.point.x, msg.point.y],
            'heading_error_rad': math.atan2(msg.point.y, msg.point.x),
            'map_pose': self.map_pose()}) + '\n')

    def on_projection(self, msg):
        # A projection is published for both blocked and permitted commands;
        # its presence alone is not a collision event.
        self.events.write(json.dumps({'state': 'rpp_projection',
            'wall_monotonic_s': time.monotonic(), 'frame': msg.header.frame_id,
            'poses': [[p.pose.position.x, p.pose.position.y] for p in msg.poses]}) + '\n')

    def close(self, report):
        self.events.close()
        self.trajectory_file.close()
        if self.expert is not None:
            self.expert.close(report)


