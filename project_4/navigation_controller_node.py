from __future__ import annotations
from typing import Optional

import rclpy
import rclpy.time

from .disc_robot import load_disc_robot

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist
from rclpy.node import Node
from random import random
import numpy as np
import math


class NavigationController(Node): 
    def __init__(self): 
        super().__init__('navigation_controller')
        # TODO: read in robot and world files and make wall threshold and speed proportional to the radius and the resolution

        self.create_subscription(LaserScan, '/scan', self.navigate, 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_v = 1.5
        self.angle_kv = 1.0
        self.kv = .4

        self.wall_threshold = 0.27

    def navigate(self, scan_msg: LaserScan):

        ranges = np.array(scan_msg.ranges)
        max_ind = np.nanargmax(ranges)
        min_ind = np.nanargmin(ranges)
    
        min_dist = scan_msg.ranges[min_ind]
        max_dist = scan_msg.ranges[max_ind]

        max_angle =  scan_msg.angle_min + (scan_msg.angle_increment * max_ind)
        min_angle = scan_msg.angle_min + (scan_msg.angle_increment * min_ind)

        vel_x = max_dist
        vel_z = -max_angle 

        if vel_x == float('inf'):
            vel_x = self.max_v
        elif vel_x == float('-inf'):
            vel_x = 0.0


        
        if (min_dist == float('-inf') or min_dist <= self.wall_threshold) and abs(min_angle) < math.pi / 2:
            vel_z = min_angle / 2
            vel_x = 0.0
            # angle_p = 0.5


        twist = Twist()
        twist.linear.x = min(vel_x * self.kv, self.max_v)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angle_kv * vel_z

        self.get_logger().info(f'vel=({twist.linear.x})')
        self.get_logger().info(f'z=({twist.angular.z})')

        self.twist_publisher.publish(twist)
    

def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()
    rclpy.spin(nav_controller)

    nav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
