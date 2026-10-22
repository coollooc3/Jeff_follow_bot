"""Evidence gates: missing monitoring cannot pass, and collisions remain latched."""

import sys
import time
from pathlib import Path
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from gazebo_msgs.msg import ContactsState, ContactState
from nav_msgs.msg import OccupancyGrid
from obstacle_scenario import (ObstacleScenario, SIZE, path_intersects_box, region_values,
                               cell_indices, BLOCKED_GOAL_CENTER, BLOCKED_GOAL_SIZE)


class ObstacleEvidenceTests(unittest.TestCase):
    def grid(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 40
        grid.info.resolution = 0.1
        grid.info.origin.position.x = -3.0
        grid.info.origin.position.y = -1.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 1600
        return grid

    def test_rectangular_region_uses_its_own_depth(self):
        grid = self.grid()
        ix, iy = cell_indices(grid, -2.0, 1.25)
        grid.data[iy * grid.info.width + ix] = 100
        self.assertNotIn(100, region_values(grid, BLOCKED_GOAL_CENTER, BLOCKED_GOAL_SIZE))

    def test_obstacle_elsewhere_does_not_prove_goal_cell_occupied(self):
        watch = ObstacleScenario(Mock(), Mock(), center=BLOCKED_GOAL_CENTER,
                                 size=BLOCKED_GOAL_SIZE, required_occupied_point=(-2.0, 1.0))
        watch.spawned = watch.static_region_free = True
        watch.spawn_requested_wall = time.monotonic()
        watch.on_contacts(ContactsState())
        grid = self.grid()
        ix, iy = cell_indices(grid, -1.9, 1.02)
        grid.data[iy * grid.info.width + ix] = 100
        watch.on_costmap(grid)
        self.assertGreater(watch.costmap_hits, 0)
        self.assertFalse(watch.evidence_ready())
        ix, iy = cell_indices(grid, -2.0, 1.0)
        grid.data[iy * grid.info.width + ix] = 100
        watch.on_costmap(grid)
        self.assertTrue(watch.evidence_ready())

    def test_blocked_goal_model_matches_its_evaluation_dimensions(self):
        sdf = Path(__file__).parents[1] / 'models/blocked_goal.sdf'
        size = ET.parse(sdf).findtext('./model/link/collision/geometry/box/size')
        self.assertEqual(tuple(map(float, size.split())), BLOCKED_GOAL_SIZE)

    def test_sparse_path_crossing_box_is_detected(self):
        self.assertTrue(path_intersects_box([[-2.0, -0.5], [-2.0, 1.0]]))

    def test_detour_path_and_empty_path_do_not_intersect_box(self):
        self.assertFalse(path_intersects_box([[-1.5, -0.5], [-1.5, 1.0]]))
        self.assertFalse(path_intersects_box([]))

    def test_touching_box_boundary_counts_as_intersection(self):
        self.assertTrue(path_intersects_box([[-2.175, -0.5], [-2.175, 1.0]]))

    def test_report_dimensions_match_physical_collision_model(self):
        sdf = Path(__file__).parents[1] / 'models/obstacle.sdf'
        size = ET.parse(sdf).findtext('./model/link/collision/geometry/box/size')
        self.assertEqual(tuple(map(float, size.split())), SIZE)

    def test_absent_sensor_is_not_zero_collision_evidence(self):
        watch = ObstacleScenario(Mock(), Mock())
        self.assertFalse(watch.result()['ok'])

    def test_robot_contact_stays_failed_after_empty_contact_messages(self):
        watch = ObstacleScenario(Mock(), Mock())
        watch.spawned = True
        watch.static_region_free = True
        watch.spawn_requested_wall = time.monotonic() - 1
        watch.first_costmap_hit_wall = watch.spawn_requested_wall + 0.1
        watch.on_contacts(ContactsState())
        self.assertTrue(watch.result()['ok'])
        contact = ContactState()
        contact.collision1_name = 'demo_obstacle::box_link::box_collision'
        contact.collision2_name = 'burger::base_link::base_collision'
        msg = ContactsState()
        msg.states = [contact]
        watch.on_contacts(msg)
        watch.on_contacts(ContactsState())
        self.assertEqual(watch.result()['robot_contact_messages'], 1)
        self.assertFalse(watch.result()['ok'])

    def test_contact_sensor_without_costmap_evidence_does_not_pass(self):
        watch = ObstacleScenario(Mock(), Mock())
        watch.spawned = True
        watch.static_region_free = True
        watch.spawn_requested_wall = time.monotonic() - 1
        watch.on_contacts(ContactsState())
        self.assertFalse(watch.result()['ok'])
        watch.first_costmap_hit_wall = time.monotonic()
        self.assertTrue(watch.result()['ok'])

    def test_clear_plan_must_arrive_after_obstacle_costmap_evidence(self):
        watch = ObstacleScenario(Mock(), Mock())
        watch.spawned = True
        watch.spawn_requested_wall = 1.0
        watch.first_costmap_hit_wall = 2.0
        detour = [[-1.5, -0.5], [-1.5, 1.0]]
        watch.observe_plan(detour, 1.5)
        self.assertIsNone(watch.first_clear_plan_wall)
        watch.observe_plan(detour, 2.1)
        self.assertEqual(watch.first_clear_plan_wall, 2.1)


if __name__ == '__main__':
    unittest.main()
