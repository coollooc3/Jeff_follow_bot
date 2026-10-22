#!/usr/bin/env python3
"""Send, inspect or cancel native Nav2 Actions in the isolated simulation."""
import argparse
import json
import math
import os
import time

STATES = ('UNKNOWN', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'SUCCEEDED', 'CANCELED', 'ABORTED')


def require_domain():
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')


def goal_values(values):
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('goal requires finite x, y, yaw (metres, radians)')
    return [float(values[0]), float(values[1]), math.atan2(math.sin(values[2]), math.cos(values[2]))]


def stopped(now, requested, odom, command):
    # Receipt ages, not a certificate of physical stopping or source freshness.
    if odom is None or command is None:
        return False
    return (requested <= odom[0] <= now and requested <= command[0] <= now and
            now - odom[0] < 0.5 and now - command[0] < 0.5 and
            abs(odom[1]) < 0.005 and abs(odom[2]) < 0.02 and
            abs(command[1]) < 1e-4 and abs(command[2]) < 1e-4)


def retain_status(previous, incoming):
    # Native status and retained session history can arrive in either order.
    # A UUID never returns from a terminal state to execution.
    return previous if previous is not None and previous >= incoming else incoming


class TaskClient:
    def __init__(self, node):
        from action_msgs.msg import GoalStatusArray
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        self.node = node
        self.states = {}
        self.history = {}
        self.odom = self.command = None
        self.action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.cancel_service = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.subscriptions = [
            node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                self.on_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(GoalStatusArray, '/leap_nav_demo/task_history',
                self.on_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data),
            node.create_subscription(Twist, '/cmd_vel', self.on_command, qos_profile_sensor_data)]

    def on_status(self, msg):
        for item in msg.status_list:
            identity = bytes(item.goal_info.goal_id.uuid).hex()
            previous = self.history.get(identity)
            state = retain_status(previous.status if previous else None, item.status)
            if state == item.status:
                self.history[identity] = item
            self.states[identity] = STATES[state]

    def on_odom(self, msg):
        self.odom = (time.monotonic(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_command(self, msg):
        self.command = (time.monotonic(), msg.linear.x, msg.angular.z)

    def spin(self, seconds):
        import rclpy
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0, end - time.monotonic())))

    def wait(self, future, seconds=5):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            self.spin(0.05)
        if not future.done():
            raise TimeoutError('ROS service response timed out; task state is unknown, query status')
        return future.result()

    def send(self, values):
        from nav2_msgs.action import NavigateToPose
        from nav_recording import pose
        goal = NavigateToPose.Goal()
        goal.pose = pose(self.node, *goal_values(values))
        handle = self.wait(self.action.send_goal_async(goal))
        return {'accepted': handle.accepted, 'goal_id': bytes(handle.goal_id.uuid).hex(),
                'note': 'Acceptance is not arrival; use status to observe the terminal result.'}

    def cancel(self):
        from action_msgs.srv import CancelGoal
        if not self.cancel_service.wait_for_service(timeout_sec=3):
            raise TimeoutError('Nav2 cancel service unavailable')
        requested = time.monotonic()
        # ROS zero UUID + zero timestamp cancels all active NavigateToPose goals.
        response = self.wait(self.cancel_service.call_async(CancelGoal.Request()))
        identities = [bytes(item.goal_id.uuid).hex() for item in response.goals_canceling]
        if response.return_code != 0:
            raise RuntimeError('Cancel rejected: ' + str(response.return_code))
        if not identities:
            return {'canceling': [], 'result': 'nothing_to_cancel', 'stop_verified': False}
        deadline = requested + 3
        while time.monotonic() < deadline:
            if (all(self.states.get(identity) == 'CANCELED' for identity in identities) and
                    stopped(time.monotonic(), requested, self.odom, self.command)):
                return {'canceling': identities, 'result': 'CANCELED', 'stop_verified': True}
            self.spin(0.05)
        raise TimeoutError('Cancel requested, but CANCELED and fresh zero motion were not confirmed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    goal = commands.add_parser('goal', help='submit map x y yaw and return after acceptance')
    goal.add_argument('pose', nargs=3, type=float)
    commands.add_parser('status', help='read native Action states; history is session scoped')
    commands.add_parser('cancel', help='cancel active NavigateToPose goals and check stopping')
    args = parser.parse_args()
    try:
        require_domain()
        if args.command == 'goal':
            goal_values(args.pose)
    except ValueError as exc:
        parser.error(str(exc))
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    node = rclpy.create_node('leap_nav_operator', parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        client = TaskClient(node)
        if not client.action.wait_for_server(timeout_sec=3):
            raise TimeoutError('Nav2 Action unavailable; start run_manual and wait for READY')
        client.spin(0.5)
        if args.command == 'goal':
            report = client.send(args.pose)
            ok = report['accepted']
        elif args.command == 'cancel':
            report, ok = client.cancel(), True
        else:
            report, ok = {'server_available': True, 'goals': client.states,
                          'note': 'No observed goals does not prove an idle or stopped robot.'}, True
        print(json.dumps(report, allow_nan=False), flush=True)
        return 0 if ok else 1
    except (Exception, KeyboardInterrupt) as exc:
        print(json.dumps({'error': type(exc).__name__, 'detail': str(exc)}), flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
