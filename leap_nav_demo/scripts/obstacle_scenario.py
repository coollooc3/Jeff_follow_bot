"""Obstacle insertion and evidence checks for the simulation tasks."""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


CENTER = (-2.0, 0.25)
SIZE = (0.35, 0.35, 0.50)
BLOCKED_GOAL_CENTER = (-2.0, 1.02)
BLOCKED_GOAL_SIZE = (0.35, 0.10, 0.50)


def path_intersects_box(poses, center=CENTER, size=SIZE):
    """Intersect segments, so a sparse path cannot jump over the box unnoticed."""
    if not poses:
        return False
    segments = list(zip(poses, poses[1:])) or [(poses[0], poses[0])]
    for start, end in segments:
        enter, leave = 0.0, 1.0
        for axis in (0, 1):
            low, high = center[axis] - size[axis] / 2, center[axis] + size[axis] / 2
            delta = end[axis] - start[axis]
            if abs(delta) < 1e-12:
                if not low <= start[axis] <= high:
                    break
            else:
                t0, t1 = sorted(((low - start[axis]) / delta, (high - start[axis]) / delta))
                enter, leave = max(enter, t0), min(leave, t1)
                if enter > leave:
                    break
        else:
            return True
    return False


def cell_indices(grid, x, y):
    """Reject unsupported or out-of-map coordinates instead of sampling another cell."""
    info = grid.info
    if (grid.header.frame_id != 'map' or info.resolution <= 0 or
            len(grid.data) != info.width * info.height or abs(info.origin.orientation.z) > 1e-6):
        raise ValueError('unsupported map geometry')
    ix = math.floor((x - info.origin.position.x) / info.resolution)
    iy = math.floor((y - info.origin.position.y) / info.resolution)
    if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
        raise ValueError('obstacle region outside map')
    return ix, iy


def region_values(grid, center=CENTER, size=SIZE):
    """Read the rectangular box region with a 5 cm sampling margin."""
    cx, cy = center
    half_x, half_y = size[0] / 2 + 0.05, size[1] / 2 + 0.05
    x0, y0 = cell_indices(grid, cx - half_x, cy - half_y)
    x1, y1 = cell_indices(grid, cx + half_x, cy + half_y)
    info = grid.info
    return [grid.data[y * info.width + x] for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]


def estimated_box_clearance(map_pose, center=CENTER, size=SIZE):
    dx = max(abs(map_pose[0] - center[0]) - size[0] / 2, 0.0)
    dy = max(abs(map_pose[1] - center[1]) - size[1] / 2, 0.0)
    return math.hypot(dx, dy) - 0.13


class ObstacleScenario:
    def __init__(self, node, log, *, center=CENTER, size=SIZE, model_file='obstacle.sdf',
                 required_occupied_point=None):
        self.node, self.log = node, log
        self.center, self.size, self.model_file = center, size, model_file
        self.required_occupied_point = required_occupied_point
        self.required_point_cost = None
        self.contacts_received = 0
        self.last_contact_time = None
        self.max_contact_gap = 0.0
        self.collisions = 0
        self.costmap_hits = 0
        self.costmap_received = False
        self.static_region_free = False
        self.min_estimated_clearance = None
        self.spawn_requested_wall = None
        self.spawned = False
        self.first_costmap_hit_wall = None
        self.first_clear_plan_wall = None
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions = [
            node.create_subscription(ContactsState, '/demo/obstacle_contacts', self.on_contacts,
                                     qos_profile_sensor_data),
            node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, map_qos),
            node.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos),
        ]

    def on_contacts(self, msg):
        now = time.monotonic()
        if self.last_contact_time is not None:
            self.max_contact_gap = max(self.max_contact_gap, now - self.last_contact_time)
        self.last_contact_time = now
        self.contacts_received += 1
        pairs = [[s.collision1_name, s.collision2_name] for s in msg.states
                 if any(name.startswith('burger::') for name in (s.collision1_name, s.collision2_name))]
        if pairs:
            self.collisions += 1
            self.log.event('injected_obstacle_contact', pairs=pairs)

    def on_costmap(self, msg):
        self.costmap_received = True
        self.costmap_hits = sum(value >= 99 for value in region_values(msg, self.center, self.size))
        if self.required_occupied_point is not None:
            ix, iy = cell_indices(msg, *self.required_occupied_point)
            self.required_point_cost = msg.data[iy * msg.info.width + ix]
        if self.spawn_requested_wall is not None and self.costmap_hits and self.first_costmap_hit_wall is None:
            self.first_costmap_hit_wall = time.monotonic()
            self.log.event('obstacle_detected_in_costmap', hits=self.costmap_hits)

    def on_map(self, msg):
        self.static_region_free = all(value == 0 for value in region_values(msg, self.center, self.size))

    def wait_for_clear_region(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.static_region_free and self.costmap_received and self.costmap_hits == 0:
                self.log.event('pre_injection_region_clear')
                return
        raise RuntimeError('need known-free original map and clear live costmap before injection')

    def spawn(self):
        client = self.node.create_client(SpawnEntity, '/spawn_entity')
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError('spawn service unavailable')
            request = SpawnEntity.Request()
            request.name = 'demo_obstacle'
            request.xml = (Path(get_package_share_directory('leap_nav_demo')) /
                           'models' / self.model_file).read_text()
            request.initial_pose.position.x, request.initial_pose.position.y = self.center
            request.initial_pose.position.z = self.size[2] / 2
            request.initial_pose.orientation.w = 1.0
            request.reference_frame = 'world'
            self.spawn_requested_wall = time.monotonic()
            self.log.event('obstacle_spawn_requested', center_m=self.center, size_m=self.size)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError('obstacle spawn failed or timed out')
        finally:
            self.node.destroy_client(client)
        self.spawned = True
        self.log.event('obstacle_spawned', center_m=self.center, size_m=self.size)

    def evidence_ready(self):
        point_ok = (self.required_occupied_point is None or
                    self.required_point_cost is not None and self.required_point_cost >= 99)
        return (self.spawned and self.contacts_received > 0 and point_ok and
                self.first_costmap_hit_wall is not None and self.static_region_free)

    def prepare(self):
        self.spawn()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.evidence_ready():
                return
        raise RuntimeError('need free original map, occupied live costmap and active contact sensor')

    def observe_plan(self, poses, received_wall):
        if (self.spawned and poses and received_wall is not None and
                self.first_costmap_hit_wall is not None and
                received_wall >= self.first_costmap_hit_wall and self.first_clear_plan_wall is None and
                not path_intersects_box(poses, self.center, self.size)):
            self.first_clear_plan_wall = received_wall
            self.log.event('post_injection_plan_avoids_box', plan_receive_wall_s=received_wall)

    def observe(self, map_pose):
        if map_pose is None:
            return
        clearance = estimated_box_clearance(map_pose, self.center, self.size)
        if self.min_estimated_clearance is None or clearance < self.min_estimated_clearance:
            self.min_estimated_clearance = clearance

    def result(self):
        age = time.monotonic() - self.last_contact_time if self.last_contact_time is not None else None
        monitor_ok = (self.contacts_received > 0 and age is not None and age < 0.5 and
                      self.max_contact_gap < 0.5)
        return {
            'center_m': self.center, 'size_m': self.size, 'static_map_region_free': self.static_region_free,
            'required_occupied_point_m': self.required_occupied_point,
            'required_point_cost': self.required_point_cost,
            'last_costmap_occupied_cells': self.costmap_hits,
            'contact_messages': self.contacts_received, 'robot_contact_messages': self.collisions,
            'contact_monitor_fresh_and_continuous': monitor_ok,
            'max_contact_receive_gap_wall_s': self.max_contact_gap,
            'min_tf_circle_to_box_clearance_m': self.min_estimated_clearance,
            'spawned': self.spawned,
            'first_costmap_hit_delay_from_request_s': (
                self.first_costmap_hit_wall - self.spawn_requested_wall
                if self.first_costmap_hit_wall is not None else None),
            'first_clear_plan_delay_from_request_s': (
                self.first_clear_plan_wall - self.spawn_requested_wall
                if self.first_clear_plan_wall is not None else None),
            'ok': self.evidence_ready() and monitor_ok and self.collisions == 0,
            'limits': 'Contacts with injected box only; TF circle clearance is an estimate, not ground truth.',
        }
