from __future__ import annotations
from typing import Optional

import rclpy
import rclpy.time

from .disc_robot import load_disc_robot
from sensor_msgs.msg import LaserScan, PointCloud
from .util import line_intersection_test, point_intersection_test, circle_intersection_test, distance
from geometry_msgs.msg import Twist, PoseStamped, Pose
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

        self.create_subscription(LaserScan, '/scan', self._laser_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._navigate, 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.execute_plan) # execute plan every 0.1 seconds
        self.create_timer(0.1, self.update_pose) # update pose


        self.wall_threshold = 0.27
        self.world = dict()
        self.radius = 0.2 # hardcoded for now
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.position = (0.8, 3.0)

        self.sample_count = 5000
        self.candidate_points = [] # points sampled
        self.prm_graph = dict() # linked list graph
        self.leaves = deque()
        self.source = None
        self.target = None
        self.connection_dist = 0.5 # min dist for graph
        self.plan = None

    def update_pose(self):
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'base_link',
        #         'world',
        #         self.get_clock().now().to_msg())
        #     self.position = (t.position.x, t.position.y)
        # except TransformException as e:
        #     self.get_logger().info(
        #         f'Could not transform base_link to world: {e}')
        #     return
        pass

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

    def _laser_callback(self, scan_msg: LaserScan):
        pass

    def _navigate(self, goal: PoseStamped):
        '''Run PRM to create navigation plan'''
        self.get_logger().info(f'Navigating to {goal.pose.position.x}, {goal.pose.position.y}')
        self.plan = None
        self.source = (self.position[0], self.position[1])
        self.target = (goal.pose.position.x, goal.pose.position.y)
        if not self._validate_point(self.target): # determine if reachable
            self.get_logger().info(f'Requested position unreachable, ignoring.')
            return
        self.plan = self._create_plan()
        self.get_logger().info(f'Plan created {self.plan}')

    def _create_plan(self):
        # Run PRM
        self.get_logger().info(f'Building plan...')
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

    def _update_graph(self):
        '''Update PRM graph and return if points added'''
        updated = False
        breadth = len(self.leaves)
        
        for _ in range(breadth):
            leaf = self.leaves.pop()
            for candidate in self.candidate_points:
                if distance(candidate[0], candidate[1], leaf[0], leaf[1]) <= self.connection_dist:
                    self._add_node(candidate, from_node=leaf)
                    updated = True
        return updated

    def _build_graph(self):
        '''Build PRM graph and return true if possible route'''
        self.get_logger().info(f'Building graph...')
        self.prm_graph = dict()
        self._add_node(self.source)
        while self.target not in self.prm_graph.keys(): # haven't reached goal
            if not self._update_graph(): # update graph and return false if no more reachable points
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
        # msg = Twist()
        # if self.plan:
            # publish
            # self.twist_publisher.publish(msg)
        # do nothing
        return



def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()
    rclpy.spin(nav_controller)

    nav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
