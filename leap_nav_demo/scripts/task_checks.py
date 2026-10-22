"""Scenario contracts and sampled stop observations, independent of ROS."""

import math


SCENARIOS = ('baseline', 'obstacle', 'sudden_obstacle', 'cancel', 'blocked_goal', 'blocked_route')


class StopProbe:
    def __init__(self, requested_wall, origin_xy):
        if not all(math.isfinite(value) for value in (requested_wall, *origin_xy)):
            raise ValueError('stop request needs finite time and odometry position')
        self.requested_wall = requested_wall
        self.origin_xy = origin_xy
        self.first_stopped_wall = None
        self.max_distance_m = 0.0
        self.invalid_samples = 0

    def observe(self, received_wall, xy, linear, angular):
        if received_wall <= self.requested_wall:
            return
        if not all(math.isfinite(value) for value in (received_wall, *xy, linear, angular)):
            self.invalid_samples += 1
            return
        self.max_distance_m = max(self.max_distance_m,
                                 math.hypot(xy[0] - self.origin_xy[0], xy[1] - self.origin_xy[1]))
        if self.first_stopped_wall is None and abs(linear) < 0.005 and abs(angular) < 0.02:
            self.first_stopped_wall = received_wall

    def result(self):
        return {'request_wall_monotonic_s': self.requested_wall,
                'request_odom_xy_m': list(self.origin_xy),
                'first_stopped_after_request_s': (self.first_stopped_wall - self.requested_wall
                    if self.first_stopped_wall is not None else None),
                'max_odom_displacement_after_request_m': self.max_distance_m,
                'invalid_samples': self.invalid_samples}


def task_failures(report):
    """An expected failure passes its scenario only with the required evidence."""
    failures = []

    def require(condition, reason):
        if not condition:
            failures.append(reason)

    scenario = report['scenario']
    require(scenario in SCENARIOS, 'unknown_scenario')
    no_commands = not report['command_observed'] and report['nonzero_commands_received'] == 0
    require(report['last_command_zero'] or (scenario == 'blocked_goal' and no_commands),
            'final_command_not_zero')
    require(report['odom_stopped_and_fresh'], 'odom_not_stopped_or_stale')
    if scenario == 'cancel':
        cancellation = report['cancellation']
        stop = cancellation.get('stop', {})
        latency = stop.get('first_stopped_after_request_s')
        require(report['result'] == 'CANCELED', 'cancel_result_not_canceled')
        require(cancellation.get('triggered', False), 'cancel_not_triggered_while_moving')
        require(cancellation.get('result_after_request_s', math.inf) <= 2.0,
                'cancel_result_too_late')
        require(latency is not None and 0 <= latency <= 2.0, 'cancel_stop_missing_or_late')
        require(stop.get('max_odom_displacement_after_request_m', math.inf) <= 0.05,
                'excessive_post_cancel_displacement')
        require(stop.get('invalid_samples') == 0, 'invalid_cancel_stop_samples')
        require(report['plans_received'] > 0 and report['nonzero_commands_received'] > 0,
                'cancel_motion_evidence_missing')
    elif scenario == 'blocked_goal':
        require(report['result'] == 'FAILED', 'occupied_goal_not_rejected')
        require(report['planner_aborted_goal_count'] > 0, 'planner_failure_evidence_missing')
        require(report['task_wall_seconds'] <= 10.0, 'occupied_goal_rejection_too_late')
        point_cost = report['obstacle'].get('required_point_cost')
        require(point_cost is not None and point_cost >= 99,
                'occupied_goal_evidence_missing')
        require(report['nonzero_commands_received'] == 0, 'motion_command_for_occupied_goal')
        require(report['odom_path_length_m'] < 0.02, 'motion_for_occupied_goal')
    else:
        require(report['result'] == 'SUCCEEDED', 'navigation_not_succeeded')
        require(report['position_error_m'] is not None and report['position_error_m'] <= 0.25,
                'position_outside_limit')
        require(report['yaw_error_rad'] is not None and report['yaw_error_rad'] <= 0.25,
                'yaw_outside_limit')
        require(report['odom_path_length_m'] > report.get('required_path_length_m', 0.5),
                'insufficient_motion')
        require(report['plans_received'] > 0 and report['nonzero_commands_received'] > 0,
                'plan_or_motion_command_missing')
    if 'obstacle' in report:
        require(report['obstacle']['ok'], 'obstacle_evidence_failed')
    if scenario == 'sudden_obstacle':
        require(report['injection']['triggered'], 'obstacle_not_injected_while_moving')
        require(report['obstacle']['first_clear_plan_delay_from_request_s'] is not None,
                'post_obstacle_plan_missing')
    if scenario == 'blocked_route':
        from route_checks import recovery_failures
        failures.extend(recovery_failures(report))
    return failures
