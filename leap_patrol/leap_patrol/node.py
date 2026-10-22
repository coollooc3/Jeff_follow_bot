"""ROS adapter for the patrol exercise. Never publishes velocity commands."""
import json
import math
import os
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .client import PatrolClient
from .exercise_check import check_exercise
from .mission import Mission
from .route import load_route


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Use patrol.launch.py or set use_sim_time:=true')
        default_route = str(Path(get_package_share_directory('leap_patrol')) / 'config/route.yaml')
        route_file = self.declare_parameter('route_file', default_route).value
        self.points = load_route(route_file)
        self.mission = Mission(point.name for point in self.points)
        self.exercise_errors = check_exercise()
        self.distance_remaining = None
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/patrol/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = PatrolClient(self.mission, self.send_goal, self.publish_status)
        self.create_service(Trigger, '/patrol/start', self.start_cb)
        self.create_service(Trigger, '/patrol/cancel', self.cancel_cb)
        # Status remains visible even before /clock starts or while Gazebo is paused.
        self.create_timer(1.0, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.parameters_cb)
        self.publish_status()
        if self.exercise_errors:
            self.get_logger().warning('Exercise incomplete; /patrol/start will refuse. Run patrol_check first.')
        else:
            self.get_logger().info('IDLE: wait for the Nav2 session READY, then call /patrol/start.')

    def parameters_cb(self, parameters):
        changed = any(parameter.name in ('route_file', 'use_sim_time') for parameter in parameters)
        return SetParametersResult(successful=not changed,
                                   reason='Restart the node to change route or clock' if changed else '')

    def start_cb(self, request, response):
        del request
        if self.exercise_errors:
            response.success, response.message = False, 'exercise_incomplete: run patrol_check and edit Mission.on_result'
        elif not self.action.server_is_ready() or self.get_clock().now().nanoseconds == 0:
            response.success, response.message = False, 'nav2_not_ready: wait for run_manual READY'
        else:
            try:
                self.client.start()
                response.success, response.message = True, 'Start requested; observe /patrol/status for results'
            except ValueError as error:
                response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def cancel_cb(self, request, response):
        del request
        requested = self.client.cancel()
        response.success = True
        response.message = ('Cancellation requested; wait for terminal status' if requested
                            else 'No goal owned by this node; stopping was not verified')
        self.publish_status()
        return response

    def send_goal(self, index):
        point = self.points[index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = point.x, point.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(point.yaw / 2), math.cos(point.yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        token = self.client.active
        self.distance_remaining = None

        def feedback(message):
            if token is self.client.active:
                distance = float(message.feedback.distance_remaining)
                self.distance_remaining = distance if math.isfinite(distance) else None

        self.get_logger().info(f'Goal {index + 1}/{len(self.points)}: {point.name}')
        return self.action.send_goal_async(goal, feedback_callback=feedback)

    def publish_status(self):
        handle = self.client.handle
        data = dict(state=self.mission.state, current_waypoint=self.mission.current_name,
                    current_index=self.mission.index, completed=self.mission.completed,
                    total=len(self.points), reason=self.mission.reason,
                    last_nav_status=self.client.last_nav_status,
                    goal_unresolved=self.client.active is not None,
                    active_goal_uuid=None if handle is None else bytes(handle.goal_id.uuid).hex(),
                    distance_remaining=self.distance_remaining,
                    exercise_ready=not self.exercise_errors, simulation_only=True)
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def close_mission(self):
        if self.client.active is None:
            return
        self.client.cancel()
        deadline = time.monotonic() + 3.0
        while self.client.active is not None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.1)
        if self.client.active is not None:
            self.get_logger().error('Goal cancellation not confirmed. Close run_manual to end the simulation session.')


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print('Simulation requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
        return 1

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous_term = signal.signal(signal.SIGTERM, interrupted)
    node = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = PatrolManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close_mission()
            except (Exception, KeyboardInterrupt) as error:
                node.get_logger().error(f'Cancellation incomplete: {error}; close the simulation session')
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
    return 0
