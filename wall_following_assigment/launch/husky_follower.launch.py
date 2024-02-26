import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    #declare arg
    forward_speed_arg = '1.0'
    distance_to_wall_arg = '1.0'

    ground_truth_tf_publisher = Node(
         package='wall_following_assigment',
         executable='ground_truth_tf_publisher.py',
         output='screen'
    
    )
    
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

    ld = LaunchDescription()
    
    ld.add_action(ground_truth_tf_publisher)
    ld.add_action(wall_follower_node_py)
    
   
    return ld