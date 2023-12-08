from __future__ import annotations

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project_4.disc_robot import load_disc_robot


class Velocity_Translator(Node): 
    def __init__(self): 
        super().__init__('velocity_translator')
        self.create_subscription(Twist, '/cmd_vel', self.translate, 10)
        self.vl_pub = self.create_publisher(Float64, '/vl',  10)
        self.vr_pub = self.create_publisher(Float64, '/vr', 10)
        self.declare_parameter('robot', 'robots/normal.robot')
        p = self.get_parameter('robot')
        self.robot = load_disc_robot(p.value)

        self.l = self.robot['wheels']['distance']

    def translate(self, twist_msg):
        # linear velocity = angular velocity * radius of curve ? 
        v_r = Float64()
        v_l = Float64()
        
        if twist_msg.angular.z == 0:
            v_r.data = twist_msg.linear.x
            v_l.data = twist_msg.linear.x     
        else: 
            R = twist_msg.linear.x / twist_msg.angular.z
            x = twist_msg.linear.x

            z = twist_msg.angular.z

            v_r.data = z * (R + (self.l / 2))
            v_l.data = z * (R - (self.l / 2))

        # self.get_logger().info('Velocity Translator=(%s, %s)' % (v_r, v_l))

        self.vr_pub.publish(v_r)
        self.vl_pub.publish(v_l)
        
def main(args=None):
    rclpy.init(args=args)
    velocity_translator = Velocity_Translator()
    rclpy.spin(velocity_translator)

    velocity_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
