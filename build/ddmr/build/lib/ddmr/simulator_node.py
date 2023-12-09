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
from rclpy.node import Node
from random import random

from tf2_ros.buffer import Buffer, tf2
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
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
        self.create_timer(self.dt, self._update_pose)
        self.create_timer(self.error_rate, self._error)

        self.broadcaster = TransformBroadcaster(self)
        self.init_static_frames()
        self.tf_laser_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_laser_buffer, self)

        self.laser_rate = self.robot['laser']['rate']
        self.laser_count : int = int(self.robot['laser']['count'])
        self.laser_angle_min : float = float(self.robot['laser']['angle_min'])
        self.laser_angle_max : float = float(self.robot['laser']['angle_max'])
        self.laser_range_min : float = float(self.robot['laser']['range_min'])
        self.laser_range_max : float = float(self.robot['laser']['range_max'])
        self.laser_error : float = float(self.robot['laser']['error_variance']) ** .5
        self.laser_fail : float = float(self.robot['laser']['fail_probability'])

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.obst_pub = self.create_publisher(PointCloud, '/obs', 10)
        self.create_timer(self.laser_rate, self.publish_scan)


    # returns xy of the laser
    def laser(self) -> np.ndarray:
        laser_offset = np.array([np.cos(self.pose[2]), np.sin(self.pose[2])]) * 0.5 * self.robot['body']['radius']
        laser2 = self.pose[:2] + laser_offset
        return laser2




    # given a heading and a line segement, returns the intersection point of the laser within the segement
    def heading_interect(self, heading: np.ndarray , p1: np.ndarray, p2: np.ndarray) ->  Optional[np.ndarray]:

        laser = self.laser()
        line = p2-p1

        # check if lines are parallel
        if np.cross(heading, line) == 0:
            return None
 
        n_heading = norm(heading)
        v1 = laser - p1
        v3 = np.array([-n_heading[1], n_heading[0]])
        t1 = np.cross(line, v1) / np.dot(line, v3)
        t2 = np.dot(v1, v3) / np.dot(line, v3)

        # checks if intersection is within bounds
        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            return laser + t1 * n_heading

        return None


    # checks along all four edges of cell if the laser scan intersects
    def cell_intersection(self, heading: np.ndarray, row: int, col: int) -> Optional[Point32]:

        min_range = float('+inf')
        min_p = None

        x = col * self.world['resolution']
        y = row * self.world['resolution']

        bottom_l = np.array([x, y])
        bottom_r = np.array([x+self.world['resolution'], y])
        top_l = np.array([x, y+self.world['resolution']])
        top_r = np.array([x+self.world['resolution'], y+self.world['resolution']])

        points = [
            self.heading_interect(heading, top_r, top_l),
            self.heading_interect(heading, bottom_r, bottom_l),
            self.heading_interect(heading, bottom_l, top_l),
            self.heading_interect(heading, bottom_r, top_r),
        ]

        laser = self.laser()

        for p in points: 
            if p is not None:
                dist = np.linalg.norm(laser - p)
                if dist < min_range:
                    min_range = dist 
                    min_p = Point32(x=p[0], y=p[1])

        return min_p

    def angle_scan(self, robot_angle: float) -> float:

        # if fails should exit early
        if random() < self.laser_fail:
            return float('nan')

        min_range = float('+inf')
        heading = np.array([np.cos(robot_angle), np.sin(robot_angle)])
        laser = self.laser()

        for row, col in self.world['obstacles']:
            p = self.cell_intersection(heading, row, col)
            if p is not None:
                dist = np.linalg.norm(laser - np.array([p.x, p.y]))
                min_range = min(min_range, dist)

            
        if min_range < self.laser_range_min:
            return float('-inf')
        if min_range > self.laser_range_max:
            return float('+inf')
        return min_range + np.random.normal(0.0, self.laser_error)

            

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'laser'
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        angles = np.linspace(self.laser_angle_min, self.laser_angle_max, self.laser_count) + self.pose[2]

        scan_msg.angle_min = self.laser_angle_min 
        scan_msg.angle_max = self.laser_angle_max 
        scan_msg.range_min = self.laser_range_min 
        scan_msg.range_max = self.laser_range_max 
        scan_msg.angle_increment = angles[1] - angles[0]

        ranges = []
        for a in angles:
            ranges.append(self.angle_scan(a))

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)



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

    def init_static_frames(self):
        radius = self.robot['body']['radius']
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = self.get_clock().now().to_msg()
        base_to_laser.header.frame_id = 'base_link' # parent
        base_to_laser.child_frame_id = 'laser'
        base_to_laser.transform.translation.x = 0.5*radius
        base_to_laser.transform.translation.y = 0.0
        base_to_laser.transform.translation.z = 0.0
        static_broadcaster = StaticTransformBroadcaster(self)
        static_broadcaster.sendTransform(base_to_laser)

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


    # def check_collision(self):
    #     for obstacle_row, obstacle_col in self.world['obstacles']:
    #         cell_center_x = self.world['resolution']/2 + self.world['resolution'] * obstacle_col
    #         cell_center_y = self.world['resolution']/2 + self.world['resolution'] * obstacle_row
    #         # circle box collision
    #         intersect, correction = intersection_test(
    #             self.pose[0], self.pose[1], self.robot['body']['radius'], # circle
    #             cell_center_x, cell_center_y, self.world['resolution'] # square
    #         )
    #         if intersect:
    #             self.pose[0] += correction[0]
    #             self.pose[1] += correction[1]
    #             break


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
