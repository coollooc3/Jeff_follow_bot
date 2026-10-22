"""Canonical start/goal configuration shared by simulation, task and evaluation."""
import hashlib
import json
import math

DEFAULT_START = (-2.0, -0.5, 0.0)
DEFAULT_GOAL = (-2.0, 1.0, math.pi / 2)


def make_task(scenario, start=None, goal=None):
    start = list(DEFAULT_START if start is None else start)
    goal = list(DEFAULT_GOAL if goal is None else goal)
    for pose in (start, goal):
        if len(pose) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                     and math.isfinite(v) for v in pose):
            raise ValueError('start and goal require finite x, y, yaw values')
        # Preserve valid serialized angles: repeated sin/atan2 can drift by an ULP.
        if not -math.pi <= pose[2] <= math.pi:
            pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if distance < 0.5:
        raise ValueError('navigation trials require start/goal at least 0.5m apart')
    default = start == list(DEFAULT_START) and goal == list(DEFAULT_GOAL)
    if scenario not in ('baseline', 'cancel') and not default:
        raise ValueError('obstacle fixtures require default start/goal; use baseline or cancel for custom tasks')
    return {'schema_version': 1, 'map': 'turtlebot3_world', 'start': start, 'goal': goal,
            'route': 'left_corridor' if default else 'custom',
            'required_path_length_m': min(0.5, 0.4 * distance)}


def encode_task(task):
    return (json.dumps(task, indent=2, sort_keys=True, allow_nan=False) + '\n').encode('utf-8')


def task_hash(task):
    return hashlib.sha256(encode_task(task)).hexdigest()


def read_task(path, scenario):
    task = json.loads(path.read_text())
    canonical = make_task(scenario, task['start'], task['goal'])
    if task != canonical or path.read_bytes() != encode_task(canonical):
        raise ValueError('task configuration is not canonical or was modified')
    return canonical
