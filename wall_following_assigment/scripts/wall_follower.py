#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, Buffer
from tf2_ros import TransformListener
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys
from std_msgs.msg import Float32

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt

    def update_control(self, current_error, reset_prev=False):

        if reset_prev:
            self.prev_error = 0
            self.prev_error_deriv = 0
        
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.sum_error += self.curr_error * self.dt
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        self.control = (
            self.Kp * self.curr_error +
            self.Ti * self.sum_error +
            self.Td * self.curr_error_deriv
        )
        
        self.prev_error_deriv = self.curr_error_deriv

    def get_control(self):
        return self.control
    

class WallFollowerHusky(Node):
    def __init__(self):
        super().__init__('wall_follower_husky')

        self.forward_speed = self.declare_parameter("forward_speed").value
        self.desired_distance_from_wall = self.declare_parameter("desired_distance_from_wall").value
        self.hz = 50

        self.cte_pub = self.create_publisher(Float32, '/husky/cte', QoSProfile(depth=10))
        self.cmd_pub = self.create_publisher(Twist, '/husky/cmd_vel', QoSProfile(depth=10))
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, QoSProfile(depth=10))  

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.Kp = 1.0
        self.Td = 1.0
        self.Ti = 0.001
        
        self.declare_parameter("Kp", self.Kp)
        self.declare_parameter("Td", self.Td)
        self.declare_parameter("Ti", self.Ti)

        self.pid = PID(self.Kp, self.Td, self.Ti, 1.0/self.hz)

        self.create_timer(1.0, self.dynamic_reconfigure_timer_callback)

    def dynamic_reconfigure_timer_callback(self):
        
        self.pid.Kp = self.get_parameter("Kp").value
        self.pid.Td = self.get_parameter("Td").value
        self.pid.Ti = self.get_parameter("Ti").value

    def laser_scan_callback(self, msg):
        
        closest_distance = min(msg.ranges)
        cte = Float32()
        cte.data = self.desired_distance_from_wall - closest_distance
        self.cte_pub.publish(cte)
        self.pid.update_control(cte.data)
        angular_velocity = self.pid.get_control()

        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.angular.z = angular_velocity
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    wfh = WallFollowerHusky()
    rclpy.spin(wfh)
    wfh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
