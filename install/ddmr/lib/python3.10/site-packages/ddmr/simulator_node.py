from __future__ import annotations
from typing import Optional
from geometry_msgs.msg import Point32, Pose, TransformStamped
from .disc_robot import load_disc_robot
from .util import circle_intersection_test, load_world

import rclpy
import rclpy.time
import math
import numpy as np
from std_msgs.msg import Float64, Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from random import random

from tf2_ros.buffer import tf2
from tf2_ros import TransformBroadcaster
from .util import norm, quaternion_from_euler, MAP_FREE, MAP_OBSTACLE


class Simulator(Node): 
    def __init__(self, dt: float = .1): 

        super().__init__('simulator')
        
        world_timer_period = 1
        self.declare_parameter('world', 'world/brick.world')
        world = self.get_parameter('world')
        self.get_logger().info(f'using world=({world.value})')
        self.world = load_world(world.value)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', world_timer_period)
        self.create_timer(world_timer_period, self.publish_world)

        self.declare_parameter('robot', 'robots/normal.robot')
        robot = self.get_parameter('robot')
        self.get_logger().info(f'using robot=({robot.value})')
        self.robot = load_disc_robot(robot.value)

        self.l = self.robot['wheels']['distance']

        self.dt : float = dt 
        self.pose = np.array(self.world['initial_pose']) # x, y, theta

        self._vl: float = 0.0
        self._vr: float = 0.0

        self.vl_error = self.robot['wheels']['error_variance_left'] ** .5
        self.vr_error = self.robot['wheels']['error_variance_right'] ** .5
        self.vl_error_scalar = np.random.normal(1.0, self.vl_error)
        self.vr_error_scalar = np.random.normal(1.0, self.vr_error)
        self.error_rate = self.robot['wheels']['error_update_rate']

        self.last_update  = 0

        self.create_subscription(Float64, '/vr', self.vl, 10)
        self.create_subscription(Float64, '/vl', self.vr , 10)
        self.pose_publisher = self.create_publisher(Pose2D, '/pose', 10)
        self.create_timer(self.dt, self._update_pose)
        self.create_timer(self.error_rate, self._error)

        self.broadcaster = TransformBroadcaster(self)


    def publish_world(self):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'world'
        # Set the map properties
        map_msg.info.width = self.world['width']
        map_msg.info.height = self.world['height']
        map_msg.info.resolution = self.world['resolution']  # Set the resolution of each grid cell
        map_msg.info.origin.position.x = 0.0 # Lower left corner is at origin
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0

        map_msg.data = self.world['map']
        # self.get_logger().info('Map String: (%s) | Length: (%s)' % (self.world['map_str'], len(self.world['map'])))
        # self.get_logger().info('map2d: (%s)' % (self.world['map_2d']))
        # self.get_logger().info('Height: (%s) | Width: (%s)' % (self.world['width'], self.world['height']))


        self.map_pub.publish(map_msg)

    def _error(self):
        self.vl_error_scalar = np.random.normal(1.0, self.vl_error)
        self.vr_error_scalar = np.random.normal(1.0, self.vr_error)

    def update_frame(self):
        world_to_base = TransformStamped()
        world_to_base.header.stamp = self.get_clock().now().to_msg()
        world_to_base.header.frame_id = 'world' # parent
        world_to_base.child_frame_id = 'base_link'
        # translation
        world_to_base.transform.translation.x = self.pose[0] 
        world_to_base.transform.translation.y = self.pose[1] 
        world_to_base.transform.translation.z = 0.0
        # rotation
        q = quaternion_from_euler(0, 0, self.pose[2])
        world_to_base.transform.rotation.x = q[0]
        world_to_base.transform.rotation.y = q[1]
        world_to_base.transform.rotation.z = q[2]
        world_to_base.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(world_to_base)

    def _update_pose(self):
        if self.last_update > 1:
            self._vl = 0.0
            self._vr = 0.0
            self.last_update = 0
            return

        self.last_update += self.dt

        x, y, a = self.pose
        potential_pose = self.pose.copy()

        if self._vr == self._vl:
            potential_pose += np.array([
                self._vl * self.dt * np.cos(self.pose[2]),
                self._vr * self.dt * np.sin(self.pose[2]),
                0.0
            ], dtype=float)
        else:
            w = (self._vr - self._vl) / self.l
            r = (self.l / 2.0) * ((self._vr + self._vl) / (self._vr - self._vl))
            #       ( x - Rsin(a) )
            # icc = ( y + Rcos(a) )
            #       (      0      )
            icc = np.array([
                x - r*np.sin(a),
                y + r*np.cos(a),
                0.0,
            ])

            b = icc + np.array([
                0.0,
                0.0,
                w*self.dt
            ])

            T = np.array([
                [np.cos(w*self.dt), -np.sin(w*self.dt), 0.0],
                [np.sin(w*self.dt), np.cos(w*self.dt), 0.0],
                [0.0, 0.0, 1.0]
            ])

            # x'   ( cos(w*dt)   -sin(w*dt)    0 )   ( x - icc_x )   (icc_x)
            # y' = ( sin(w*dt)    cos(w*dt)    0 ) * ( x + icc_x ) + (icc_y) 
            # a'   (    0             0        1 )   (     a     )   (w *dt)
            potential_pose = (T @ (self.pose - icc)) + b

        # Make sure no collision
        if not self.collision_test(potential_pose):
            self.pose = potential_pose # valid pose
        
        # Publish new pose
        msg = Pose2D()
        msg.x, msg.y, msg.theta = self.pose[0], self.pose[1], self.pose[2]
        self.pose_publisher.publish(msg)

        # Update frame
        self.update_frame()
        
    def collision_test(self, potential_pose):
        for obstacle_row, obstacle_col in self.world['obstacles']:
            cell_center_x = self.world['resolution']/2 + self.world['resolution'] * obstacle_col
            cell_center_y = self.world['resolution']/2 + self.world['resolution'] * obstacle_row
            # circle box collision
            intersect = circle_intersection_test(
                potential_pose[0], potential_pose[1], self.robot['body']['radius'], # circle
                cell_center_x, cell_center_y, self.world['resolution'] # square
            )
            if intersect:
                return True
        return False


    def vl(self, vl_msg: Float64):
        self.last_update = 0
        self._vl = vl_msg.data * self.vl_error_scalar

    def vr(self, vr_msg: Float64):
        self.last_update = 0
        self._vr = vr_msg.data * self.vr_error_scalar

    
def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
