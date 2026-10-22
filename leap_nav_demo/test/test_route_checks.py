"""A free goal can be unreachable; retry success cannot erase an invalid first phase."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from route_checks import GATE_SIZE, ROOM_HALF_SPAN, connectivity, cell_index
from task_checks import task_failures
from test_task_checks import report


def ring_grid():
    grid = {'frame': 'map', 'width': 11, 'height': 11, 'resolution': .1,
            'origin': [-.5, -.5, 0.], 'data': [0] * 121}
    for i in range(3, 8):
        for x, y in ((3, i), (7, i), (i, 3), (i, 7)):
            grid['data'][y * 11 + x] = 100
    return grid


def recovered_report():
    value = report('blocked_route')
    value['goal_map_pose'] = [-2., 1., 1.57]
    state = {'reachable': True, 'start_cost': 0, 'goal_cost': 0}
    closed = {'result': 'FAILED', 'action_id': 'first', 'goal': value['goal_map_pose'],
              'task_wall_seconds': .2, 'planner_aborted_goals': 1, 'nonzero_commands': 0,
              'odom_path_length_m': .001, 'stopped_and_fresh': True}
    value['route_recovery'] = {'original_connectivity': dict(state),
        'closed_connectivity': {**state, 'reachable': False}, 'opened_connectivity': dict(state),
        'gate_deleted': True, 'same_nav2_session': True,
        'gate_evidence_before_removal': {'ok': True},
        'closed_attempt': closed, 'retry_attempt': {'result': 'SUCCEEDED', 'action_id': 'second',
                                                   'goal': list(value['goal_map_pose'])}}
    return value


class RouteChecksTests(unittest.TestCase):
    def test_physical_room_and_gate_match_evaluation_geometry(self):
        root = Path(__file__).parents[1] / 'models'
        gate = ET.parse(root / 'route_gate.sdf')
        self.assertEqual(tuple(map(float, gate.findtext('./model/link/collision/geometry/box/size').split())), GATE_SIZE)
        room = ET.parse(root / 'route_room.sdf')
        poses = {link.attrib['name']: tuple(map(float, link.findtext('pose').split()[:2]))
                 for link in room.findall('./model/link')}
        self.assertEqual(poses, {'west': (-ROOM_HALF_SPAN, 0.), 'east': (ROOM_HALF_SPAN, 0.),
                                 'south': (0., -ROOM_HALF_SPAN)})

    def test_closed_ring_blocks_free_goal_even_with_diagonal_search(self):
        result = connectivity(ring_grid(), [0., 0.], [.4, 0.])
        self.assertFalse(result['reachable'])
        self.assertEqual((result['start_cost'], result['goal_cost']), (0, 0))

    def test_opening_exit_restores_connectivity(self):
        grid = ring_grid()
        for x in range(4, 7):
            grid['data'][7 * 11 + x] = 0
        self.assertTrue(connectivity(grid, [0., 0.], [.4, 0.])['reachable'])

    def test_unknown_escape_must_not_be_misreported_as_disconnected(self):
        grid = ring_grid()
        grid['data'][7 * 11 + 5] = -1
        self.assertTrue(connectivity(grid, [0., 0.], [.4, 0.])['reachable'])

    def test_occupied_goal_and_unknown_start_are_not_free_endpoints(self):
        for point, cost in (([.4, 0.], 100), ([0., 0.], -1)):
            grid = ring_grid()
            grid['data'][cell_index(grid, point)] = cost
            self.assertFalse(connectivity(grid, [0., 0.], [.4, 0.])['reachable'])

    def test_malformed_rotated_and_outside_grids_fail_explicitly(self):
        for changed in ({'data': [0]}, {'resolution': 0.}, {'origin': [0., 0., .1]}, {'frame': 'odom'}):
            grid = ring_grid()
            grid.update(changed)
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                connectivity(grid, [0., 0.], [.4, 0.])
        with self.assertRaises(ValueError):
            connectivity(ring_grid(), [5., 5.], [.4, 0.])

    def test_closed_failure_and_same_goal_retry_pass(self):
        self.assertEqual(task_failures(recovered_report()), [])

    def test_final_success_cannot_hide_wrong_first_action_result_or_motion(self):
        for changed in ({'result': 'CANCELED'}, {'result': 'SUCCEEDED'}, {'planner_aborted_goals': 0},
                        {'nonzero_commands': 1}, {'odom_path_length_m': .03},
                        {'stopped_and_fresh': False}, {'task_wall_seconds': 10.1}):
            value = recovered_report()
            value['route_recovery']['closed_attempt'].update(changed)
            with self.subTest(changed=changed):
                self.assertTrue(task_failures(value))

    def test_closed_goal_occupancy_or_missing_topology_cannot_substitute_for_no_route(self):
        for changed in ({'reachable': True}, {'goal_cost': 100}, {'start_cost': -1}):
            value = recovered_report()
            value['route_recovery']['closed_connectivity'].update(changed)
            self.assertIn('closed_route_not_proven', task_failures(value))

    def test_gate_removal_session_reset_and_goal_change_are_checked(self):
        for key, changed in (('gate_deleted', False), ('same_nav2_session', False)):
            value = recovered_report()
            value['route_recovery'][key] = changed
            self.assertTrue(task_failures(value))
        value = recovered_report()
        value['route_recovery']['retry_attempt']['goal'] = [0., 0., 0.]
        self.assertIn('retry_goal_changed', task_failures(value))

    def test_result_identity_and_open_route_evidence_are_required(self):
        value = recovered_report()
        value['route_recovery']['retry_attempt']['action_id'] = 'first'
        self.assertIn('distinct_action_results_missing', task_failures(value))
        value = recovered_report()
        value['route_recovery']['opened_connectivity']['reachable'] = False
        self.assertIn('opened_route_not_proven', task_failures(value))


if __name__ == '__main__':
    unittest.main()
