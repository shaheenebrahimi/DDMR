import rclpy.node
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn, SetPen
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float32MultiArray
import math

import sys

class ActionNode(Node):
    def __init__(self, actions):
        super().__init__('action')
        self.actions = actions
        actions.insert(0, [5.544445, 5.544445, actions[0][0], actions[0][1]])
        
        
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle'+str(turtle_num)+'/pose',
            self.pose_callback,
            self.queue_size)
        self.current_pose = {'x':5.44445, 'y':5.44445, 'theta':0}

        self.twist_publisher = self.create_publisher(Twist, '/turtle'+str(turtle_num)+'/cmd_vel', self.queue_size)
        self.timer_period = 0.1 # second
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    
    def pose_callback(self, pose_msg):
        # self.get_logger().info("x: " + str(self.current_pose['x']) + " y: " + str(self.current_pose['y']) + " theta: " + str(self.current_pose['theta']))
        self.current_pose['x'] = pose_msg.x
        self.current_pose['y'] = pose_msg.y
        self.current_pose['theta'] = pose_msg.theta
    
    def timer_callback(self):
        
        # goal_angle = math.atan2((self.actions[3]-self.actions[1]),(self.actions[2]-self.actions[0]))
        if len(self.actions) > 0:
            self.get_logger().info("X: " + str(self.current_pose['x']) + " | " + "Y: " + str(self.current_pose['y']) + " | " + "0: " + str(self.current_pose['theta']))
        
            line_angle = math.atan2((self.actions[0][3]-self.actions[0][1]),(self.actions[0][2]-self.actions[0][0]))
            angle_to_start = math.atan2((self.actions[0][1]-self.current_pose['y']),(self.actions[0][0]-self.current_pose['x']))
            self.get_logger().info("Current ANGLE: " + str(self.current_pose['theta']) + "| ANGLE TO START PT: "+ str(angle_to_start) +"| GOAL ANGLE: " + str(line_angle))
            self.get_logger().info("ACTION: " +  str(self.actions))
            distance_to_goal = math.sqrt((self.actions[0][2]-self.current_pose['x'])**2 + (self.actions[0][3]-self.current_pose['y'])**2)
            self.get_logger().info("DISTANCE TO GOAL: " +  str(distance_to_goal))
            if distance_to_goal > 0.1:
                if abs(line_angle-self.current_pose['theta']) > 0.01:
                    self.turn(line_angle, self.current_pose['theta'])
                else:
                    self.move(distance_to_goal)
            else:
                self.get_logger().info("Done with action!")
                self.get_logger().info("ACTION: " +  str(self.actions))
                if len(self.actions) > 1:
                    if self.actions[0][2] == self.actions[1][0]:
                        self.actions[1][0] = self.current_pose['x']
                    if self.actions[0][3] == self.actions[1][1]:
                        self.actions[1][1] = self.current_pose['y']
                    self.actions = self.actions[1:]
                else:
                    self.actions = []
                    self.stop_moving()
            
        self.i+=1

    def listener_callback(self, msg):
        # Get line segment data (startpoint, endpoint)
        # if abs(current point - startpoint) > eps:

        #  penup

        #  rotate to face start point 

        #  move to start point 
        # else
        #  pendown
        # rotate to face endpoint
        # move to endpoint

        # Action structure: (endpoint, absolute angle towards target, penup/pendown)

        # Rotate in place: (current pose (x,y), angle start to end, penup)
        # Move towards endpoint: (endpoint, current pose theta, penup/pendown)

        # convert to Twist
        # publish Twist 10x/sec
        self.actions.append(msg.data)
        self.get_logger().info("I Hear you!" + " data: ")
        # self.turn(math.atan2((msg.data[3]-msg.data[1]),(msg.data[2]-msg.data[0]))) 
        # self.move(msg.data[1], msg.data[3])

    def turn(self, goal_angle, curr_angle):
        self.get_logger().info("turning...")
        accel = 0.5 * abs(goal_angle-curr_angle)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = accel 

        # while self.current_pose['theta'] < goal_angle:
            # self.get_logger().info("current theta:" + str(self.current_pose['theta']) + " | Goal Theta: " + str(goal_angle))
        self.twist_publisher.publish(twist)

    def move(self, dist_to_goal):
        self.get_logger().info("moving...")
        twist = Twist()
        twist.linear.x = 0.5 * dist_to_goal
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Move turtle as wanted distance
        # while math.sqrt((x_final-self.current_pose['x'])**2 + (y_final-self.current_pose['y'])**2) > 0.01:
        self.twist_publisher.publish(twist)

    def stop_moving(self):
        self.get_logger().info("stopping...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Move turtle as wanted distance
        # while math.sqrt((x_final-self.current_pose['x'])**2 + (y_final-self.current_pose['y'])**2) > 0.01:
        self.twist_publisher.publish(twist)
