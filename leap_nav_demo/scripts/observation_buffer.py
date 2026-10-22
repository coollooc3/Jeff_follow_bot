"""Common causal observation selection for demonstrations and simulation policy trials."""
from collections import Counter, deque
import time
import rclpy
from tf2_ros import TransformException
from expert_data import INPUT_LIMITS, build_sample, latest_past


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def planar_pose(transform):
    import math
    p, q = transform.transform.translation, transform.transform.rotation
    return [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]


def query_pose(buffer, target, source, query_s):
    """Nonblocking exact lookup; latest TF is diagnostic context, never a pose fallback."""
    diagnostic = {'target': target, 'source': source, 'query_sim_s': query_s}
    try:
        transform = buffer.lookup_transform(target, source, rclpy.time.Time(seconds=query_s))
        pose = planar_pose(transform)
        diagnostic.update(status='available', returned_stamp_s=stamp(transform))
        return pose, diagnostic
    except (TransformException, ValueError) as exc:
        diagnostic.update(status='unavailable', exception_type=type(exc).__name__, error=str(exc))
        # This lookup has no timeout and does not spin the executor or permit motion.
        try:
            latest = buffer.lookup_transform(target, source, rclpy.time.Time())
            diagnostic['latest_available_stamp_s'] = stamp(latest)
        except (TransformException, ValueError) as latest_exc:
            diagnostic['latest_error'] = str(latest_exc)
        return None, diagnostic


class ObservationBuffer:
    def __init__(self, node, buffer, goal):
        self.node, self.buffer, self.goal = node, buffer, goal
        self.latest = {'scan': None, 'odom': None, 'plan': None}
        self.history = {kind: deque(maxlen=64) for kind in self.latest}
        self.ids = Counter()

    def save_input(self, kind, message, **fields):
        self.ids[kind] += 1
        record = {'kind': kind, 'id': self.ids[kind], 'stamp_s': stamp(message),
                  'received_wall_s': time.monotonic(), 'frame': message.header.frame_id, **fields}
        self.latest[kind] = record
        self.history[kind].append(record)
        return record

    def on_scan(self, message):
        self.save_input('scan', message, angle_min=message.angle_min, angle_increment=message.angle_increment,
                        range_min=message.range_min, range_max=message.range_max,
                        time_increment=message.time_increment, scan_time=message.scan_time,
                        ranges=list(message.ranges))

    def on_odom(self, message):
        self.save_input('odom', message, child_frame=message.child_frame_id,
                        velocity=[message.twist.twist.linear.x, message.twist.twist.angular.z])

    def on_plan(self, message):
        self.save_input('plan', message, points=[[p.pose.position.x, p.pose.position.y] for p in message.poses])

    def fresh_older_odometry(self, action, newest):
        """Bounded causal history using the same freshness limits as sample validation."""
        sim_limit, wall_limit = INPUT_LIMITS['odom']
        return sorted((record for record in self.history['odom']
                       if latest_past([record], action) is not None
                       and record['stamp_s'] < newest['stamp_s']
                       and 0 <= action['sim_s'] - record['stamp_s'] <= sim_limit
                       and 0 <= action['wall_s'] - record['received_wall_s'] <= wall_limit),
                      key=lambda record: (record['stamp_s'], record['received_wall_s']), reverse=True)

    def sample(self, action, active, following, require_follow_path=True):
        selected = {kind: latest_past(records, action, scan=(kind == 'scan'))
                    for kind, records in self.history.items()}
        scan, odom = selected['scan'], selected['odom']
        pose, laser_pose = None, None
        diagnostics = {}
        if odom is not None and scan is not None:
            pose, diagnostics['map_to_base'] = query_pose(self.buffer, 'map', 'base_footprint', odom['stamp_s'])
            if pose is not None:
                laser_pose, diagnostics['base_to_laser'] = query_pose(self.buffer, 'base_footprint', scan['frame'], 0.)
        sample = build_sample(scan, odom, selected['plan'], action, pose, laser_pose,
                              self.goal, active, following, require_follow_path=require_follow_path)
        selection = {'newest_past_id': odom['id'] if odom else None, 'used_history': False, 'failed_queries': []}
        if pose is None and odom is not None and 'map_to_base' in diagnostics:
            selection['failed_queries'].append({'odom_id': odom['id'], **diagnostics['map_to_base']})
            # Do not hide an invalid input or a different guard failure by selecting history.
            if sample['reasons'] == ['transform_missing']:
                for previous in self.fresh_older_odometry(action, odom):
                    previous_pose, diagnostic = query_pose(self.buffer, 'map', 'base_footprint', previous['stamp_s'])
                    if previous_pose is None:
                        selection['failed_queries'].append({'odom_id': previous['id'], **diagnostic})
                        continue
                    odom, pose = previous, previous_pose
                    diagnostics['map_to_base'] = diagnostic
                    laser_pose, diagnostics['base_to_laser'] = query_pose(self.buffer, 'base_footprint', scan['frame'], 0.)
                    sample = build_sample(scan, odom, selected['plan'], action, pose, laser_pose,
                                          self.goal, active, following, require_follow_path=require_follow_path)
                    selection['used_history'] = True
                    break
        sample['latest_received_input_ids'] = {kind: record['id'] if record else None
                                                for kind, record in self.latest.items()}
        sample['pose_query_sim_s'] = odom['stamp_s'] if odom else None
        sample['transform_diagnostics'] = diagnostics
        sample['odom_selection'] = selection
        return sample
