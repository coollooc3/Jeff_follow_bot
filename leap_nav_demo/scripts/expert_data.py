"""Causal receipt-time checks and explicit-frame features for recorded demonstrations."""
import math

SECTORS = 36
INPUT_LIMITS = {'scan': (0.2, 0.4), 'odom': (0.2, 0.4), 'plan': (2.0, 3.0)}


def latest_past(records, action, scan=False):
    eligible = []
    for record in records:
        source = record['stamp_s']
        increment = record.get('time_increment', 0.0) if scan else 0.0
        end = source + max(0, len(record.get('ranges', [])) - 1) * increment
        if (all(math.isfinite(v) for v in (source, end, record['received_wall_s'], increment))
                and increment >= 0 and end <= action['sim_s']
                and record['received_wall_s'] <= action['wall_s']):
            eligible.append(record)
    return max(eligible, key=lambda r: (r['stamp_s'], r['received_wall_s'])) if eligible else None


def scan_features(scan):
    minimum, maximum = scan['range_min'], scan['range_max']
    if (not all(math.isfinite(v) for v in (minimum, maximum, scan['angle_min'], scan['angle_increment']))
            or not 0 <= minimum < maximum or scan['angle_increment'] == 0 or not scan['ranges']):
        raise ValueError('invalid scan metadata')
    values = [None] * SECTORS
    finite_counts, censored_counts, invalid_counts = ([0] * SECTORS for _ in range(3))
    for index, value in enumerate(scan['ranges']):
        angle = scan['angle_min'] + index * scan['angle_increment']
        sector = min(SECTORS - 1, int(((angle + math.pi) % (2 * math.pi)) * SECTORS / (2 * math.pi)))
        if value is not None and math.isfinite(value) and minimum <= value <= maximum:
            finite_counts[sector] += 1
            values[sector] = value if values[sector] is None else min(values[sector], value)
        elif value == math.inf:
            censored_counts[sector] += 1
        else:
            invalid_counts[sector] += 1
    for index in range(SECTORS):
        if values[index] is None and censored_counts[index]:
            values[index] = maximum
    return {'sector_min_m': values, 'finite_ray_counts': finite_counts,
            'positive_infinite_ray_counts': censored_counts, 'invalid_ray_counts': invalid_counts}


def body_vector(point, pose):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return [cosine * dx + sine * dy, -sine * dx + cosine * dy]


def path_reference(points, pose, lookahead=0.35):
    if len(points) < 2 or not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('invalid global path')
    nearest = min(range(len(points)), key=lambda i: math.hypot(points[i][0] - pose[0], points[i][1] - pose[1]))
    remaining = lookahead
    for first, second in zip(points[nearest:], points[nearest + 1:]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        if length > 0 and length >= remaining:
            ratio = remaining / length
            return body_vector([first[0] + ratio * (second[0] - first[0]),
                                first[1] + ratio * (second[1] - first[1])], pose)
        remaining -= length
    return body_vector(points[-1], pose)


def build_sample(scan, odom, plan, action, pose, laser_pose, goal, active, following, *, require_follow_path=True):
    """Use preceding fresh inputs; planner-only policy mode does not require FollowPath."""
    reasons = []
    result = {'action': action, 'input_ids': {}, 'input_ages': {}, 'features': {},
              'task_active': active, 'follow_path_active': following}
    if not active:
        reasons.append('task_inactive')
    if require_follow_path and not following:
        reasons.append('follow_path_not_active')
    for name, record in (('scan', scan), ('odom', odom), ('plan', plan)):
        sim_limit, wall_limit = INPUT_LIMITS[name]
        if record is None:
            reasons.append(name + '_missing')
            continue
        sim_age = action['sim_s'] - record['stamp_s']
        wall_age = action['wall_s'] - record['received_wall_s']
        result['input_ids'][name] = record['id']
        result['input_ages'][name] = {'source_sim_s': sim_age, 'receive_wall_s': wall_age}
        if (not math.isfinite(sim_age) or not 0 <= sim_age <= sim_limit or
                not math.isfinite(wall_age) or not 0 <= wall_age <= wall_limit):
            reasons.append(name + '_future_or_stale')
    if not all(math.isfinite(v) for v in (action['sim_s'], action['wall_s'], *action['velocity'])):
        reasons.append('invalid_action')
    if pose is None or laser_pose is None:
        reasons.append('transform_missing')
    elif not all(math.isfinite(v) for v in (*pose, *laser_pose)):
        reasons.append('invalid_transform')
    else:
        target = body_vector(goal[:2], pose)
        yaw_error = math.atan2(math.sin(goal[2] - pose[2]), math.cos(goal[2] - pose[2]))
        result['features'].update(goal_xy_base_m=target,
            goal_yaw_error_sin_cos=[math.sin(yaw_error), math.cos(yaw_error)],
            map_pose_at_odom_stamp=pose, laser_pose_in_base=laser_pose)
    if scan is not None:
        try:
            increment = scan.get('time_increment', 0.0)
            if not math.isfinite(increment) or increment < 0:
                raise ValueError('invalid scan ray timing')
            if scan['stamp_s'] + (len(scan['ranges']) - 1) * increment > action['sim_s']:
                reasons.append('scan_future_rays')
            compressed = scan_features(scan)
            result['features']['scan_laser_frame'] = compressed
            if any(v is None for v in compressed['sector_min_m']):
                reasons.append('unknown_scan_sectors')
        except ValueError:
            reasons.append('invalid_scan')
    if odom is not None:
        result['features']['odom_velocity'] = odom['velocity']
        if (odom['frame'] != 'odom' or odom['child_frame'] != 'base_footprint' or
                not all(math.isfinite(v) for v in odom['velocity'])):
            reasons.append('invalid_odom_frame_or_velocity')
    if plan is not None:
        if plan['frame'] != 'map':
            reasons.append('invalid_plan_frame')
        elif pose is not None and all(math.isfinite(v) for v in pose):
            try:
                result['features']['path_reference_xy_base_m'] = path_reference(plan['points'], pose)
            except ValueError:
                reasons.append('invalid_plan')
    result['candidate'] = not reasons
    result['reasons'] = reasons
    return result
