#!/usr/bin/env python3
"""Autonomous opp driver for f1tenth_gym_ros 2-car mode.

Listens to /opp_racecar/odom, follows a fixed lane CSV via speed-adaptive
pure pursuit (PD steering), publishes /opp_drive. Hardcoded to opp namespace
because that's what gym_bridge wires up.

Adapted from f1tenth-racing-stack-ICRA22 dummy_car/scripts/dummy_car_node.py,
trimmed to the speed-adaptive interp branch and our 4-col CSV format
(x, y, yaw, speed_ratio) used by pure_pursuit_multi.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from ament_index_python.packages import get_package_share_directory


def safe_idx(length, idx, delta):
    return (idx + delta + length) % length


class OppDummy(Node):
    def __init__(self):
        super().__init__('opp_dummy_node')

        # Parameters
        self.declare_parameter('csv_path', '')
        # If csv_path is empty, fall back to <pure_pursuit_multi share>/waypoints/lanes/<csv_name>.csv
        self.declare_parameter('csv_name', 'lane_optimal')

        self.declare_parameter('overwrite_speed', True)
        self.declare_parameter('speed', 4.0)        # m/s, fixed when overwrite_speed=True
        self.declare_parameter('vel_scale', 0.8)    # multiplier for csv-derived speed

        # Speed-adaptive lookahead: L = minL + (maxL-minL) / Lscale * speed
        self.declare_parameter('minL', 0.5)
        self.declare_parameter('maxL', 1.5)
        self.declare_parameter('Lscale', 7.0)

        # Speed-adaptive PD steering: P = maxP - (maxP-minP) / Pscale * speed
        self.declare_parameter('minP', 0.5)
        self.declare_parameter('maxP', 0.7)
        self.declare_parameter('Pscale', 7.0)
        self.declare_parameter('D', 5.0)
        self.declare_parameter('max_steer', 0.36)   # rad

        self.declare_parameter('interpScale', 20)   # interpolation samples between waypoints
        self.declare_parameter('visualize', True)

        # Topic params (defaults match f1tenth_gym_ros 2-car mode)
        self.declare_parameter('odom_topic', '/opp_racecar/odom')
        self.declare_parameter('drive_topic', '/opp_drive')

        # Resolve CSV path
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        if not csv_path:
            csv_name = self.get_parameter('csv_name').get_parameter_value().string_value
            try:
                ppm_share = get_package_share_directory('pure_pursuit_multi')
                csv_path = os.path.join(ppm_share, 'waypoints', 'lanes', f'{csv_name}.csv')
            except Exception as e:
                self.get_logger().error(
                    f"csv_path empty and pure_pursuit_multi share not found: {e}")
                raise
        self.csv_path = csv_path

        # Load 4-col CSV: x, y, yaw, speed_ratio
        wps = np.loadtxt(self.csv_path, delimiter=',')
        if wps.ndim != 2 or wps.shape[1] != 4:
            raise RuntimeError(
                f"expected Nx4 CSV (x,y,yaw,speed_ratio), got {wps.shape} from {self.csv_path}")
        self.traj_x = wps[:, 0]
        self.traj_y = wps[:, 1]
        self.traj_v = wps[:, 3]  # speed ratio in [0,1]
        self.traj_pos = np.column_stack((self.traj_x, self.traj_y))
        self.num_pts = len(wps)

        self.curr_vel = 0.0
        self.prev_steer_error = 0.0
        self.target_point = None

        # Topics
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.path_pub = self.create_publisher(Marker, '/opp_global_path', 10)
        self.target_pub = self.create_publisher(Marker, '/opp_waypoint', 10)

        self.get_logger().info(
            f"opp_dummy: following {self.num_pts} waypoints from {os.path.basename(self.csv_path)}, "
            f"odom='{odom_topic}', drive='{drive_topic}'")

    def odom_callback(self, msg: Odometry):
        self.curr_vel = float(msg.twist.twist.linear.x)

        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        cyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cur_pos = np.array([cx, cy])

        # Nearest waypoint
        dists = np.linalg.norm(self.traj_pos - cur_pos, axis=1)
        curr_idx = int(np.argmin(dists))

        # Adaptive lookahead distance (grows with speed)
        L = self._L_for_speed(self.curr_vel)

        # Walk forward until we leave the lookahead circle, then interpolate the segment
        seg_end = curr_idx
        while dists[seg_end] <= L:
            seg_end = (seg_end + 1) % self.num_pts
            if seg_end == curr_idx:  # full loop guard
                break
        seg_begin = safe_idx(self.num_pts, seg_end, -1)

        interp_n = self.get_parameter('interpScale').get_parameter_value().integer_value
        xs = np.linspace(self.traj_x[seg_begin], self.traj_x[seg_end], interp_n)
        ys = np.linspace(self.traj_y[seg_begin], self.traj_y[seg_end], interp_n)
        vs = np.linspace(self.traj_v[seg_begin], self.traj_v[seg_end], interp_n)
        xy = np.column_stack((xs, ys))
        i_best = int(np.argmin(np.abs(np.linalg.norm(xy - cur_pos, axis=1) - L)))

        target = xy[i_best]
        target_v_ratio = float(vs[i_best])
        L_actual = float(np.linalg.norm(cur_pos - target))
        if L_actual < 1e-3:
            return  # avoid div-by-zero

        # Transform to vehicle frame
        cs, sn = math.cos(cyaw), math.sin(cyaw)
        local_x =  cs * (target[0] - cx) + sn * (target[1] - cy)
        local_y = -sn * (target[0] - cx) + cs * (target[1] - cy)

        # Pure pursuit curvature
        gamma = 2.0 / (L_actual ** 2)
        error = gamma * local_y
        steer = self._steer_for_speed(self.curr_vel, error)

        # Speed: either fixed or csv-derived
        if self.get_parameter('overwrite_speed').get_parameter_value().bool_value:
            speed = float(self.get_parameter('speed').get_parameter_value().double_value)
        else:
            vel_scale = self.get_parameter('vel_scale').get_parameter_value().double_value
            # Map ratio in [0,1] to [0, max_speed_in_csv_terms*scale]; here we treat ratio*scale as m/s
            speed = float(target_v_ratio * vel_scale)

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.speed = speed
        drive.drive.steering_angle = steer
        self.drive_pub.publish(drive)

        self.target_point = target
        if self.get_parameter('visualize').get_parameter_value().bool_value:
            self._publish_path_marker()
            self._publish_target_marker()

    def _L_for_speed(self, speed):
        minL = self.get_parameter('minL').get_parameter_value().double_value
        maxL = self.get_parameter('maxL').get_parameter_value().double_value
        Lscale = self.get_parameter('Lscale').get_parameter_value().double_value
        return (maxL - minL) / Lscale * max(speed, 0.0) + minL

    def _steer_for_speed(self, speed, error):
        minP = self.get_parameter('minP').get_parameter_value().double_value
        maxP = self.get_parameter('maxP').get_parameter_value().double_value
        Pscale = self.get_parameter('Pscale').get_parameter_value().double_value
        D = self.get_parameter('D').get_parameter_value().double_value
        max_steer = self.get_parameter('max_steer').get_parameter_value().double_value

        cur_P = maxP - max(speed, 0.0) * (maxP - minP) / Pscale
        d_err = error - self.prev_steer_error
        self.prev_steer_error = error
        steer = cur_P * error + D * d_err
        return float(np.clip(steer, -max_steer, max_steer))

    def _publish_path_marker(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'opp_global_path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.pose.orientation.w = 1.0
        for i in range(self.num_pts + 1):
            j = i % self.num_pts
            p = Point(); p.x = float(self.traj_x[j]); p.y = float(self.traj_y[j]); p.z = 0.05
            m.points.append(p)
            c = ColorRGBA(); c.r = 1.0; c.g = 0.5; c.b = 0.0; c.a = 1.0
            m.colors.append(c)
        self.path_pub.publish(m)

    def _publish_target_marker(self):
        if self.target_point is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'opp_target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.target_point[0])
        m.pose.position.y = float(self.target_point[1])
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 1.0
        self.target_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OppDummy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
