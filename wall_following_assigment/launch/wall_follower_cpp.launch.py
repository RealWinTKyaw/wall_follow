#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
    
    #declare arg
    forward_speed_arg = '0.8'
    distance_to_wall_arg = '1.0'



    #node 
   
    
    wall_follower_node_cpp = Node(package='wall_following_assigment',
         executable='wall_follower',
         name='wall_follower_node_cpp',
         output='screen',
         parameters=[{"forward_speed": forward_speed_arg,
                      "desired_distance_from_wall": distance_to_wall_arg,
                      }],
         remappings=[('/husky/cmd_vel', '/husky_velocity_controller/cmd_vel_unstamped')]
                      )

    ld = LaunchDescription()
    
    
    ld.add_action(wall_follower_node_cpp)

    return ld
    