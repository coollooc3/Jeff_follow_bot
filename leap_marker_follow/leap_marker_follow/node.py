"""ROS adapter: camera/robot inputs -> continuous policy -> guarded cmd_vel."""
from collections import deque
import json
import math
import signal
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .controller import Command, Follower, Settings
from .reference import Reference, teach
from .safety import InputAge, obstacle_blocked, odom_valid, scan_points


class MarkerFollow(Node):
    def __init__(self):
        super().__init__('marker_follow')
        defaults = dict(vars(Settings()), marker_id=0,
                        reference_file='~/.ros/leap_marker_follow/reference.json',
                        input_timeout=.4, front_margin_m=.18, side_margin_m=.04)
        self.p = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        self.follower = Follower(Settings(**{name: self.p[name] for name in vars(Settings())}))
        if (type(self.p['marker_id']) is not int or not 0 <= self.p['marker_id'] < 50
                or not all(math.isfinite(self.p[k]) for k in
                           ('input_timeout', 'front_margin_m', 'side_margin_m'))
                or not .1 <= self.p['input_timeout'] <= .6
                or not .10 <= self.p['front_margin_m'] <= .35
                or not .02 <= self.p['side_margin_m'] <= .15):
            raise ValueError('Invalid marker ID, input timeout or obstacle margins')
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV with the aruco module is required')
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = (cv2.aruco.ArucoDetector(dictionary)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        self.dictionary = dictionary
        self.reference = None
        try:
            reference = Reference.load(self.p['reference_file'])
            if reference.marker_id != self.p['marker_id']:
                raise ValueError('Reference marker ID does not match configuration')
            self.reference = reference
            self.follower.set_reference(reference.side)
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().info(f'Reference unavailable; capture it before enabling: {error}')
        self.ages = {name: InputAge() for name in ('image', 'scan', 'odom')}
        self.samples = deque(maxlen=16)
        self.image_size = None
        self.points = None
        self.stationary = False
        self.stop_until = self.obstacle_until = 0.
        self.output_conflict = False
        self.cycle = 0
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug', 1)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.image_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self.enable_cb)
        self.create_service(Trigger, '~/capture_reference', self.reference_cb)
        self.add_on_set_parameters_callback(self.parameter_cb)
        self.create_timer(.05, self.tick)
        self.get_logger().info('Starts disabled. Teach at 25cm once, then enable continuous following.')

    def parameter_cb(self, parameters):
        changed = any(p.name in self.p for p in parameters)
        return SetParametersResult(successful=not changed,
                                   reason='Edit the YAML and restart to change startup parameters' if changed else '')

    def accept(self, name, message, now, valid=True):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        return self.ages[name].accept(now, stamp, age, valid, self.p['input_timeout'])

    def image_cb(self, message):
        now = time.monotonic()
        try:
            frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('JPEG decode failed')
            corners, ids, _ = (self.detector.detectMarkers(frame) if self.detector is not None
                               else cv2.aruco.detectMarkers(frame, self.dictionary))
            valid = time.monotonic()-now <= self.follower.settings.target_timeout
            if not self.accept('image', message, now, valid):
                return
            height, width = frame.shape[:2]
            self.image_size = (width, height)
            values = [] if ids is None else ids.flatten().tolist()
            targets = [corner for value, corner in zip(values, corners) if value == self.p['marker_id']]
            if len(targets) == 1:
                points = np.asarray(targets[0]).reshape(4, 2)
                side = float(np.linalg.norm(points-np.roll(points, -1, axis=0), axis=1).mean()/width)
                offset = float(2*points.mean(axis=0)[0]/width-1)
                self.follower.observe(now, side, offset)
                if self.follower.visible:
                    self.samples.append((now, side, offset, width, height))
            else:
                self.follower.observe(now)
            if self.debug_pub.get_subscription_count():
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                debug = Image(header=message.header, height=height, width=width,
                              encoding='bgr8', is_bigendian=0, step=width*3, data=frame.tobytes())
                self.debug_pub.publish(debug)
        except (ValueError, cv2.error) as error:
            self.ages['image'].issue = 'decode_error'
            self.follower.observe(now)
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def scan_cb(self, message):
        now = time.monotonic()
        points = scan_points(message.ranges, message.angle_min, message.angle_max,
                             message.angle_increment, message.range_min, message.range_max,
                             message.header.frame_id)
        if self.accept('scan', message, now, points is not None):
            self.points = points

    def odom_cb(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        if self.accept('odom', message, time.monotonic(),
                       odom_valid(values, message.header.frame_id, message.child_frame_id)):
            self.stationary = abs(twist.linear.x) <= .01 and abs(twist.angular.z) <= .05

    def health_reason(self, now):
        for name, age in self.ages.items():
            reason = age.reason(now, self.p['input_timeout'])
            if reason:
                return f'{name}_{reason}'
        if self.reference is None:
            return 'reference_missing'
        if self.image_size != (self.reference.width, self.reference.height):
            return 'reference_resolution_mismatch'
        return ''

    def other_writer(self):
        return self.count_publishers(self.command_pub.topic_name) > 1

    def publish_command(self, command=Command()):
        message = Twist()
        message.linear.x, message.angular.z = float(command.linear), float(command.angular)
        self.command_pub.publish(message)

    def disable(self):
        self.follower.set_enabled(False)
        self.stop_until = time.monotonic()+.3
        self.publish_command()

    def enable_cb(self, request, response):
        if not request.data:
            self.disable()
            response.success, response.message = True, 'Following disabled; zero command sent'
            return response
        now = time.monotonic()
        self.output_conflict = self.other_writer()
        reason = 'output_conflict' if self.output_conflict else self.health_reason(now)
        if not reason and (self.follower.count < 3 or not self.follower.visible
                           or self.follower.last_seen is None
                           or now-self.follower.last_seen > self.follower.settings.target_timeout):
            reason = 'target_not_confirmed'
        response.success = not bool(reason)
        response.message = reason or 'Continuous following enabled'
        if response.success:
            self.follower.set_enabled(True)
        return response

    def reference_cb(self, request, response):
        del request
        now = time.monotonic()
        try:
            if self.follower.enabled or self.other_writer():
                raise ValueError('Disable following and stop other cmd_vel publishers before teaching')
            if (not self.stationary or any(age.reason(now, self.p['input_timeout']) for age in self.ages.values())
                    or not self.follower.visible):
                raise ValueError('Fresh camera/scan/odom and a stationary robot with visible target required')
            reference = teach(list(self.samples), now, self.p['marker_id'])
            reference.save(self.p['reference_file'])
            self.reference = reference
            self.follower.set_reference(reference.side)
            response.success, response.message = True, 'Saved image reference; operator placement defines the 25cm gap'
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def tick(self):
        now = time.monotonic()
        self.cycle += 1
        if self.cycle % 4 == 0:
            self.output_conflict = self.other_writer()
        if self.output_conflict and self.follower.enabled:
            self.disable()
            self.get_logger().error('Other cmd_vel publisher detected; following disabled')
        reason = 'output_conflict' if self.output_conflict else self.health_reason(now)
        was_enabled = self.follower.enabled
        command = self.follower.step(now, reason)
        if was_enabled and not self.follower.enabled:
            self.disable()
        if self.follower.enabled and not reason:
            if self.points is not None and obstacle_blocked(self.points, command.linear, command.angular,
                                                           self.p['front_margin_m'], self.p['side_margin_m']):
                self.obstacle_until = now+.2
            if now < self.obstacle_until:
                command = self.follower.step(now, 'obstacle_stop')
        if self.follower.enabled or now < self.stop_until:
            self.publish_command(command)
        if self.cycle % 4 == 0:
            data = dict(enabled=self.follower.enabled,
                        state='paused' if self.follower.enabled and command.reason not in ('following', 'holding')
                        else command.reason,
                        reason=reason or command.reason, reference_ready=self.reference is not None,
                        nominal_front_gap_cm=25, confirmations=self.follower.count,
                        horizontal_offset=self.follower.offset,
                        marker_size_ratio=None if self.reference is None or self.follower.side is None
                        else self.follower.side/self.reference.side,
                        command=dict(linear=command.linear, angular=command.angular),
                        input_ages={name: None if age.good is None else now-age.good
                                    for name, age in self.ages.items()})
            self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))


def main(args=None):
    # Keep the context alive until finally can send zero on Ctrl+C / launch SIGTERM.
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous_term = signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MarkerFollow()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.disable()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
