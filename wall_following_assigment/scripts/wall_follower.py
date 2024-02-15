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
        # todo: implement this
        #self.control = ???
        pass

    def get_control(self):
        return self.control
    

class WallFollowerHusky(Node):
    def __init__(self):
        super().__init__('wall_follower_husky')

        self.forward_speed = self.declare_parameter("forward_speed").value
        self.desired_distance_from_wall = self.declare_parameter("desired_distance_from_wall").value
        self.hz = 50 

        # todo: set up the command publisher to publish at topic '/husky/cmd_vel'
        # using geometry_msgs/Twist messages
        self.cmd_pub = self.create_publisher(Twist, '/husky/cmd_vel', QoSProfile(depth=10))

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spin() call, as long as a laser scan
        # message has been published in the meantime by another node
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, QoSProfile(depth=10))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control. 

        # cmd.angular.z = ???

        pass

def main(args=None):
    rclpy.init(args=args)


    wfh=WallFollowerHusky()
    rclpy.spin(wfh)
    wfh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
