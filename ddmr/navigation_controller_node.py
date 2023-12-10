from __future__ import annotations
from typing import Optional

import rclpy
import rclpy.time

from .disc_robot import load_disc_robot
from sensor_msgs.msg import LaserScan, PointCloud
from .util import line_intersection_test, circle_intersection_test, distance, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from collections import deque
from random import uniform
import numpy as np
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class NavigationController(Node): 
    def __init__(self): 
        super().__init__('navigation_controller')

        self.dt = 0.1
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._navigate, 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.execute_plan) # execute plan every 0.1 seconds
        self.create_timer(self.dt, self._transform_callback) # execute plan every 0.1 seconds

        self.world = dict()
        self.radius = 0.2 # hardcoded for now
        self.pose = (0,0,0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sample_count = 2500
        self.candidate_points = [] # points sampled
        self.prm_graph = dict() # linked list graph
        self.leaves = deque()
        self.source = None
        self.target = None
        self.connection_dist = 0.75 # min dist for graph
        self.plan = None
        self.plan_step = 1
        
    def _map_callback(self, map_msg: OccupancyGrid):
        self.world['width'] = map_msg.info.width
        self.world['height'] = map_msg.info.height
        self.world['origin'] = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.world['resolution'] = map_msg.info.resolution
        self.world['obstacles'] = []
        for index, cell in enumerate(map_msg.data):
            # gives index
            if (cell != 0):
                r = int(index / self.world['width']) 
                c = int(index % self.world['width'])

                x = self.world['resolution']/2 + c * self.world['resolution'] 
                y = self.world['resolution']/2 + r * self.world['resolution']
                self.world['obstacles'].append((x, y)) # keep track of centers of obstacles

    def _transform_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                rclpy.time.Time())
            roll, pitch, yaw = euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            self.pose = (t.transform.translation.x, t.transform.translation.y, yaw)

        except TransformException as e:
            self.get_logger().info(f'Could not transform: {e}')
            return

    def _navigate(self, goal: PoseStamped):
        '''Run PRM to create navigation plan'''
        self.get_logger().info(f'Destination set to ({goal.pose.position.x}, {goal.pose.position.y})')
        self._reset_graph()
        self.source = (self.pose[0], self.pose[1])
        self.target = (goal.pose.position.x, goal.pose.position.y)
        
        if not self._validate_point(self.target): # determine if reachable
            self.get_logger().info(f'Requested position unreachable, ignoring.')
            return
        self.plan = self._create_plan()
        self.get_logger().info(f'Plan created.')
        self.get_logger().info(f'Navigating to ({goal.pose.position.x}, {goal.pose.position.y})')

    def _reset_graph(self):
        self.plan = None
        self.plan_step = 1
        self.candidate_points = [] # points sampled
        self.prm_graph = dict() # linked list graph
        self.leaves = deque()

    def _create_plan(self):
        # Run PRM
        while True: # keep sampling and building graph until solution possible
            self._sample_points()
            if self._build_graph(): # if successful stop trying
                break
        shortest_path = self._bfs() # get shortest path solution for graph
        return shortest_path

    def _validate_point(self, point):
        if point[0] < 0 or point[0] > self.world['width'] or point[1] < 0 or point[1] > self.world['height']: # outside bounds
            return False
        for cell_x, cell_y in self.world['obstacles']:
            if circle_intersection_test(point[0], point[1], self.radius, cell_x, cell_y, self.world['resolution']): # in obstacle
                return False
        return True
    
    def _validate_edge(self, pt0_x, pt0_y, pt1_x, pt1_y):

        for cell_x, cell_y in self.world['obstacles']:
            if line_intersection_test(pt0_x, pt0_y, pt1_x, pt1_y, self.radius, cell_x, cell_y, self.world['resolution']): # in obstacle
                return False
        return True
    
    def _sample_points(self):
        self.get_logger().info(f'Sampling points...')
        self.candidate_points = []
        for _ in range(self.sample_count):
            # random sample within world from origin (bottom left) to width height
            sample = (
                uniform(self.world['origin'][0], self.world['width']*self.world['resolution']), # x
                uniform(self.world['origin'][1], self.world['height']*self.world['resolution']) # y
            )
            if self._validate_point(sample):
                self.candidate_points.append(sample)
        self.candidate_points.append(self.target)

    def _add_node(self, node, from_node=None):
        if node not in self.prm_graph:
            self.prm_graph[node] = set() # create new entry in graph
            self.leaves.append(node) # now a leaf of graph
        if from_node:
            self.prm_graph[from_node].add(node) # create edge

    def _expand_graph(self):
        '''Update PRM graph and return if points added'''
        updated = False
        breadth = len(self.leaves)
        
        for _ in range(breadth):
            leaf = self.leaves.pop()
            for candidate in self.candidate_points:
                if distance(candidate[0], candidate[1], leaf[0], leaf[1]) <= self.connection_dist:
                    if self._validate_edge(candidate[0], candidate[1], leaf[0], leaf[1]):
                        self._add_node(candidate, from_node=leaf)
                        updated = True
        return updated

    def _build_graph(self):
        '''Build PRM graph and return true if possible route'''
        self.get_logger().info(f'Building graph...')
        self.prm_graph = dict()
        self._add_node(self.source)
        while self.target not in self.prm_graph.keys(): # haven't reached goal
            if not self._expand_graph(): # update graph and return false if no more reachable points
                return False
        return True

    def _bfs(self):
        self.get_logger().info(f'Finding shortest path...')
        frontier = deque()
        frontier.append([self.source])
        visited = set()
        while len(frontier):
            path = frontier.pop()  # get the first path from the queue
            node = path[-1] # get the last node from the path
            if node == self.target: # path found
                return path
            if node not in visited:
                visited.add(node)
                for adjacent in self.prm_graph[node]:
                    frontier_path = list(path)
                    frontier_path.append(adjacent)
                    frontier.append(frontier_path)

    def execute_plan(self):
        msg = Twist() # empty message
        if self.plan and self.plan_step < len(self.plan):
            curr_x, curr_y, curr_theta = self.pose
            initial_x, initial_y = self.plan[self.plan_step-1]
            destination = self.plan[self.plan_step]
            direction = math.atan2(destination[1] - initial_y, destination[0] - initial_x)
            delta_dir = direction - curr_theta
            delta_pos = math.dist((curr_x, curr_y), destination)

            # if not facing direction
            if abs(delta_dir) > 0.025: # turn
                if delta_dir < 0:
                    delta_dir += 2 * math.pi
                turn = -1 if delta_dir < math.pi else 1
                speed = 0.5 * abs(delta_dir) + 0.025
                msg.linear.x = 0.0
                msg.angular.z = turn * speed # turn other dir

            elif delta_pos > 0.1: # move forward
                speed = 0.8 * abs(delta_dir) + 0.25
                msg.linear.x = speed
                msg.angular.z = 0.0
            else: # reached goal
                self.plan_step += 1
                if self.plan_step == len(self.plan):
                    self.get_logger().info(f'Arrived at destination!')

        # publish
        self.twist_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()
    rclpy.spin(nav_controller)

    nav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
