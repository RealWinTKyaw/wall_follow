#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    #declare arg
    forward_speed_arg = '1.0'
    distance_to_wall_arg = '1.0'

    #node 
    wall_follower_node_py = Node(
        package='wall_following_assigment',
        executable='wall_follower.py',
        name='wall_follower_node',
        output='screen',
        parameters=[{"forward_speed": forward_speed_arg,
                     "desired_distance_from_wall": distance_to_wall_arg
                     }],
        remappings=[('/husky/cmd_vel', '/husky_velocity_controller/cmd_vel_unstamped')]
    )
    
    wall_follower_node_cpp = Node(package='wall_following_assigment',
         executable='wall_follower_node',
         name='wall_follower_node_cpp',
         output='screen',
         parameters=[{"forward_speed": forward_speed_arg,
                      "desired_distance_from_wall": distance_to_wall_arg,
                      }],
         remappings=[('/husky/cmd_vel', '/husky_velocity_controller/cmd_vel_unstamped')]
                      )
    

    ld = LaunchDescription()
    ld.add_action(wall_follower_node_py)
    

    return ld
    