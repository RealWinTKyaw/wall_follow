#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from builtin_interfaces.msg import Duration
import numpy as np

class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.tfBuffer = Buffer()
        self.tf_listener = TransformListener(self.tfBuffer, self)

        self.p_map_odom1 = None
        self.q_map_odom1 = None

        self.received_odometry_to_map = False
        self.timer = self.create_timer(0.01, self.timer_callback)

    def odom1_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p_map_baselink = np.array([p.x, p.y, p.z])
        q_map_baselink = np.array([q.x, q.y, q.z, q.w])

        # Assuming that the tf2 library provides the required functionality
        transform_stamped = do_transform_pose(msg.pose, self.tf_listener.lookup_transform('map', 'husky_1/odom', rclpy.Time(0)))

        # Extract transformed position and orientation
        p_odom1_baselink = np.array([transform_stamped.pose.position.x,
                                    transform_stamped.pose.position.y,
                                    transform_stamped.pose.position.z])

        q_odom1_baselink = np.array([transform_stamped.pose.orientation.x,
                                    transform_stamped.pose.orientation.y,
                                    transform_stamped.pose.orientation.z,
                                    transform_stamped.pose.orientation.w])

        self.tf_broadcaster.sendTransform(p_odom1_baselink,
                                          q_odom1_baselink,
                                          msg.header.stamp,
                                          'husky_1/base_link',
                                          'husky_1/odom')

    def timer_callback(self):
        if not self.received_odometry_to_map:
            try:
                transform_stamped = self.tf_listener.lookup_transform('map', 'husky_1/odom', rclpy.Time(0))
                self.p_map_odom1 = transform_stamped.transform.translation
                self.q_map_odom1 = transform_stamped.transform.rotation
                self.received_odometry_to_map = True
            except Exception as e:
                pass

        if self.received_odometry_to_map:
            real_odom_sub1 = self.create_subscription(Odometry, '/husky_1/odometry/ground_truth', self.odom1_callback, 10)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
