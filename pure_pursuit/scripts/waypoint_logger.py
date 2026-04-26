#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry

home = expanduser('~')

def quaternion_to_yaw(x, y, z, w):
    term1 = 2.0 * (w * z + x * y)
    term2 = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(term1, term2)

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.last_x = None
        self.last_y = None
        self.min_dist = 0.1

        self.declare_parameter('use_sim', True)
        use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        if use_sim:
            filepath = strftime(home + '/lab5_ws/src/lab5_repo/pure_pursuit/waypoints/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv'
        else:
            filepath = strftime('/home/nvidia/f1tenth_ws/src/labs/lab5/pure_pursuit/waypoints/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv'

        self.file = open(filepath, 'w')
        self.get_logger().info(f'Saving waypoints to {filepath}')

        topic = '/ego_racecar/odom' if use_sim else '/pf/pose/odom'
        self.get_logger().info(f'Subscribing to {topic}')

        self.subscription = self.create_subscription(
            Odometry, topic, self.save_waypoint, 10)

    def save_waypoint(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        if self.last_x is not None:
            dist = np.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            if dist < self.min_dist:
                return

        self.last_x = x
        self.last_y = y

        yaw = quaternion_to_yaw(data.pose.pose.orientation.x,
                                data.pose.pose.orientation.y,
                                data.pose.pose.orientation.z,
                                data.pose.pose.orientation.w)

        speed = LA.norm(np.array([data.twist.twist.linear.x,
                                   data.twist.twist.linear.y,
                                   data.twist.twist.linear.z]), 2)

        # if data.twist.twist.linear.x > 0.:
        #     print(data.twist.twist.linear.x)

        self.file.write('%f, %f, %f, %f\n' % (x, y, yaw, speed))

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    atexit.register(lambda: node.file.close())
    print('Saving waypoints...')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()