"""Read-only recording of Nav2 commands and their preceding observed inputs."""
from collections import Counter
import json
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from expert_data import SECTORS
from observation_buffer import ObservationBuffer


def safe_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return '+inf' if value == math.inf else '-inf' if value == -math.inf else None
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    return value


class ExpertRecorder(ObservationBuffer):
    def __init__(self, node, buffer, directory, goal):
        super().__init__(node, buffer, goal)
        self.directory = directory
        self.raw = (directory / 'expert_inputs.jsonl').open('w')
        self.samples = (directory / 'expert_samples.jsonl').open('w')
        self.active, self.following = False, False
        self.executed = None
        self.counts, self.rejections = Counter(), Counter()
        self.subscriptions = [
            node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
            node.create_subscription(Twist, '/cmd_vel_nav', self.on_teacher, qos_profile_sensor_data),
            node.create_subscription(GoalStatusArray, '/follow_path/_action/status', self.on_status, 10)]
        metadata = {'schema_version': 2, 'teacher_topic': '/cmd_vel_nav', 'executed_topic': '/cmd_vel',
                    'input_selection': 'Newest source stamp already received before command callback, not after its sampled ROS time; history max 64 per input.',
                    'sector_centers_laser_rad': [-math.pi + (i + .5) * 2 * math.pi / SECTORS for i in range(SECTORS)],
                    'source_limits_sim_s': {'scan': .2, 'odom': .2, 'plan': 2.0},
                    'receive_limits_wall_s': {'scan': .4, 'odom': .4, 'plan': 3.0},
                    'path_reference_lookahead_m': .35,
                    'limits': 'Headerless command receipt approximates decision time; not exact Nav2 input synchronization. '
                              'Teacher topic is shared with behaviors; FollowPath status is an attribution hint. '
                              'Laser sectors remain in laser frame; +inf is censored at range_max, not obstacle-free truth. '
                              'Global path remains an input: this is a local-controller demonstration, not end-to-end navigation.'}
        (directory / 'expert_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')

    def save_input(self, kind, message, **fields):
        record = super().save_input(kind, message, **fields)
        self.raw.write(json.dumps(safe_json(record), allow_nan=False) + '\n')

    def on_status(self, message):
        self.following = any(item.status == GoalStatus.STATUS_EXECUTING for item in message.status_list)

    def on_executed(self, message):
        self.executed = {'received_wall_s': time.monotonic(),
                         'velocity': [message.linear.x, message.angular.z]}

    def on_teacher(self, message):
        action = {'wall_s': time.monotonic(), 'sim_s': self.node.get_clock().now().nanoseconds * 1e-9,
                  'velocity': [message.linear.x, message.angular.z]}
        sample = self.sample(action, self.active, self.following)
        sample['last_received_executed_command'] = self.executed
        self.counts['samples'] += 1
        self.counts['candidates'] += int(sample['candidate'])
        self.rejections.update(sample['reasons'])
        sample['id'] = self.counts['samples']
        self.samples.write(json.dumps(safe_json(sample), allow_nan=False) + '\n')

    def close(self, report):
        self.raw.close()
        self.samples.close()
        summary = {'counts': {name: self.counts[name] for name in ('samples', 'candidates')},
                   'rejection_counts': dict(self.rejections),
                   'input_counts': dict(self.ids), 'task_result': report['result'],
                   'episode_candidate': bool(report['ok'] and report['result'] == 'SUCCEEDED' and
                       report.get('recoveries_reported_by_nav2') == 0 and self.counts['candidates'] >= 20),
                   'limits': 'Candidate collection only, not a validated training dataset or trained policy. '
                             'Whole-scene contact and exact teacher decision-time alignment are unverified. '
                             'Split entire episodes/routes, not adjacent frames.'}
        (self.directory / 'expert_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
