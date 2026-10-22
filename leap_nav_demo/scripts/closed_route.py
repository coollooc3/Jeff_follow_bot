"""Physical U-shaped room plus removable exit, with saved map-connectivity evidence."""
import json
import math
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from obstacle_scenario import ObstacleScenario
from route_checks import GATE_SIZE, ROOM_HALF_SPAN, cell_index, connectivity


def grid_snapshot(message):
    position, q = message.info.origin.position, message.info.origin.orientation
    return {'frame': message.header.frame_id, 'width': message.info.width, 'height': message.info.height,
            'resolution': message.info.resolution,
            'origin': [position.x, position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))],
            'stamp_s': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            'data': list(message.data)}


class ClosedRoute:
    def __init__(self, node, log, directory, start, goal):
        self.node, self.log, self.directory = node, log, directory
        self.start, self.goal = start[:2], goal[:2]
        self.static_map = self.costmap = None
        self.costmap_received = None
        self.models = Path(get_package_share_directory('leap_nav_demo')) / 'models'
        self.gate_center = (self.start[0], self.start[1] + ROOM_HALF_SPAN)
        self.wall_points = [(self.start[0] - ROOM_HALF_SPAN, self.start[1]),
                            (self.start[0] + ROOM_HALF_SPAN, self.start[1]),
                            (self.start[0], self.start[1] - ROOM_HALF_SPAN)]
        self.gate = ObstacleScenario(node, log, center=self.gate_center, size=GATE_SIZE,
                                    model_file='route_gate.sdf', required_occupied_point=self.gate_center)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions = [node.create_subscription(OccupancyGrid, '/map', self.on_map, qos),
            node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, qos)]

    def on_map(self, message):
        self.static_map = grid_snapshot(message)

    def on_costmap(self, message):
        self.costmap, self.costmap_received = grid_snapshot(message), time.monotonic()

    def save(self, name, grid, proof):
        (self.directory / ('route-' + name + '.json')).write_text(json.dumps(
            {'grid': grid, 'start': self.start, 'goal': self.goal, 'proof': proof}, allow_nan=False) + '\n')
        self.log.event('route_' + name, **proof)
        return proof

    def wait_original(self):
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
            if self.static_map is not None:
                proof = connectivity(self.static_map, self.start, self.goal)
                if not proof['reachable']:
                    raise RuntimeError('original map does not connect configured start and goal')
                return self.save('original', self.static_map, proof)
        raise TimeoutError('original map unavailable')

    def prepare(self):
        data = (self.models / 'route_room.sdf').read_text()
        (self.directory / 'route_room_used.sdf').write_text(data)
        (self.directory / 'route_gate_used.sdf').write_bytes((self.models / 'route_gate.sdf').read_bytes())
        client = self.node.create_client(SpawnEntity, '/spawn_entity')
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise TimeoutError('spawn service unavailable')
            request = SpawnEntity.Request()
            request.name, request.xml, request.reference_frame = 'demo_route_room', data, 'world'
            request.initial_pose.position.x, request.initial_pose.position.y = self.start
            request.initial_pose.position.z, request.initial_pose.orientation.w = .25, 1.
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.)
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError('room spawn failed')
        finally:
            self.node.destroy_client(client)
        self.log.event('route_room_spawned')
        changed_sim = self.node.get_clock().now().nanoseconds * 1e-9
        self.gate.prepare()
        return self.wait_topology('closed', False, changed_sim)

    def wait_topology(self, name, reachable, changed_sim):
        deadline, checked_stamp = time.monotonic() + 10., None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
            grid = self.costmap
            if grid is None or grid['stamp_s'] == checked_stamp:
                continue
            age = self.node.get_clock().now().nanoseconds * 1e-9 - grid['stamp_s']
            if (grid['stamp_s'] < changed_sim or not 0 <= age <= 2.5 or
                    time.monotonic() - self.costmap_received > 2.5):
                continue
            checked_stamp = grid['stamp_s']
            walls = [grid['data'][cell_index(grid, point)] for point in self.wall_points]
            gate = grid['data'][cell_index(grid, self.gate_center)]
            proof = connectivity(grid, self.start, self.goal)
            if (all(value >= 99 for value in walls) and proof['reachable'] is reachable and
                    (gate < 99 if reachable else gate >= 99) and
                    0 <= proof['start_cost'] < 99 and 0 <= proof['goal_cost'] < 99):
                return self.save(name, grid, {**proof, 'wall_point_costs': walls,
                                             'gate_point_cost': gate, 'source_age_sim_s': age})
        raise TimeoutError('live costmap did not prove route ' + name)

    def open(self):
        client = self.node.create_client(DeleteEntity, '/delete_entity')
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise TimeoutError('delete service unavailable')
            request = DeleteEntity.Request()
            request.name = 'demo_obstacle'  # Owned gate spawned by ObstacleScenario.
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.)
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError('gate deletion failed')
        finally:
            self.node.destroy_client(client)
        self.log.event('route_gate_deleted')
        self.node.clearAllCostmaps()
        self.log.event('route_explicit_costmaps_cleared')
        changed_sim = self.node.get_clock().now().nanoseconds * 1e-9
        return self.wait_topology('opened', True, changed_sim)
