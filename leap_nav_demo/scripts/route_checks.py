"""Point-grid connectivity and the closed-exit / retry scenario contract."""
from collections import deque
import math

ROOM_HALF_SPAN = .45
GATE_SIZE = (1., .1, .5)


def cell_index(grid, point):
    origin = grid['origin']
    if (grid['frame'] != 'map' or len(origin) != 3 or abs(origin[2]) > 1e-6 or
            any(not isinstance(grid[key], int) or isinstance(grid[key], bool) for key in ('width', 'height')) or
            not all(math.isfinite(value) for value in (*origin, grid['resolution'], *point)) or
            grid['resolution'] <= 0 or grid['width'] <= 0 or grid['height'] <= 0 or
            len(grid['data']) != grid['width'] * grid['height']):
        raise ValueError('unsupported occupancy grid geometry')
    x, y = (math.floor((point[axis] - origin[axis]) / grid['resolution']) for axis in (0, 1))
    if not 0 <= x < grid['width'] or not 0 <= y < grid['height']:
        raise ValueError('point outside grid')
    return y * grid['width'] + x


def connectivity(grid, start, goal):
    """Allow unknown cells and 8-neighbors: disconnection is a conservative no-route proof.

    This is point connectivity, not a footprint-aware plan or clearance certificate.
    """
    first, last = cell_index(grid, start), cell_index(grid, goal)
    data, width, height = grid['data'], grid['width'], grid['height']
    if any(not isinstance(value, int) or isinstance(value, bool) or not -1 <= value <= 100 for value in data):
        raise ValueError('invalid occupancy values')
    result = {'reachable': False, 'start_cost': data[first], 'goal_cost': data[last],
              'visited_cells': 0, 'connectivity': '8-neighbor; cost>=99 blocked; unknown allowed'}
    if not 0 <= data[first] < 99 or not 0 <= data[last] < 99:
        return result
    pending, visited = deque([first]), {first}
    while pending:
        current = pending.popleft()
        if current == last:
            result['reachable'] = True
            break
        x, y = current % width, current // width
        for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
            nx, ny = x + dx, y + dy
            neighbor = ny * width + nx
            if (0 <= nx < width and 0 <= ny < height and neighbor not in visited
                    and data[neighbor] < 99):
                visited.add(neighbor)
                pending.append(neighbor)
    result['visited_cells'] = len(visited)
    return result


def recovery_failures(report):
    evidence = report.get('route_recovery', {})
    closed, opened = evidence.get('closed_attempt', {}), evidence.get('retry_attempt', {})
    failures = []

    def require(condition, reason):
        if not condition:
            failures.append(reason)

    for name, expected in (('original', True), ('closed', False), ('opened', True)):
        state = evidence.get(name + '_connectivity', {})
        free_endpoints = all(isinstance(state.get(key), int) and 0 <= state[key] < 99
                             for key in ('start_cost', 'goal_cost'))
        require(state.get('reachable') is expected and free_endpoints, name + '_route_not_proven')
    require(closed.get('result') == 'FAILED', 'closed_action_not_failed')
    require(closed.get('task_wall_seconds', math.inf) <= 10., 'closed_failure_too_late')
    require(closed.get('planner_aborted_goals', 0) > 0, 'closed_planner_failure_missing')
    require(closed.get('nonzero_commands') == 0 and closed.get('odom_path_length_m', math.inf) < .02,
            'motion_while_route_closed')
    require(closed.get('stopped_and_fresh') is True, 'closed_stop_unconfirmed')
    require(evidence.get('gate_evidence_before_removal', {}).get('ok') is True, 'closed_gate_evidence_failed')
    require(evidence.get('gate_deleted') is True and evidence.get('same_nav2_session') is True,
            'gate_removal_or_same_session_missing')
    require(closed.get('goal') == opened.get('goal') == report['goal_map_pose'], 'retry_goal_changed')
    require(bool(closed.get('action_id')) and bool(opened.get('action_id')) and
            closed['action_id'] != opened['action_id'], 'distinct_action_results_missing')
    require(opened.get('result') == 'SUCCEEDED', 'retry_action_not_succeeded')
    return failures
