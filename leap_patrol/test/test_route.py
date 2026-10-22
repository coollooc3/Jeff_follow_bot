import pathlib
import tempfile
import unittest

from leap_patrol.route import load_route


class RouteTests(unittest.TestCase):
    def test_default_order_and_units(self):
        points = load_route(pathlib.Path(__file__).resolve().parents[1] / 'config/route.yaml')
        self.assertEqual([point.name for point in points], ['A', 'B', 'C'])
        self.assertEqual((points[0].x, points[0].y), (-1.8, .7))
        self.assertAlmostEqual(points[1].yaw, -1.5707963267948966)

    def test_bad_route_inputs_are_rejected(self):
        for content in ('frame_id: odom\nwaypoints: []\n',
                        'frame_id: map\nwaypoints: []\n',
                        'frame_id: map\nwaypoints: [{name: A, x: .nan, y: 0, yaw: 0}]\n',
                        'frame_id: map\nwaypoints: [{name: A, x: true, y: 0, yaw: 0}]\n',
                        'frame_id: map\nwaypoints: [{name: A, x: 0, y: 0, yaw: 0}, {name: A, x: 1, y: 1, yaw: 0}]\n'):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as directory:
                path = pathlib.Path(directory) / 'route.yaml'
                path.write_text(content, encoding='utf-8')
                with self.assertRaises(ValueError):
                    load_route(path)


if __name__ == '__main__':
    unittest.main()
