"""Load an ordered route without importing ROS."""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


def load_route(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('frame_id') != 'map':
        raise ValueError('Route must use frame_id: map')
    rows = data.get('waypoints')
    if not isinstance(rows, list) or not rows:
        raise ValueError('Route needs at least one waypoint')
    points = []
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('name'), str) or not row['name'].strip():
            raise ValueError('Every waypoint needs a nonempty name')
        values = [row.get(key) for key in ('x', 'y', 'yaw')]
        if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
            raise ValueError('Waypoint x/y/yaw must be finite numbers')
        points.append(Waypoint(row['name'], *map(float, values)))
    if len({point.name for point in points}) != len(points):
        raise ValueError('Waypoint names must be unique')
    return points
