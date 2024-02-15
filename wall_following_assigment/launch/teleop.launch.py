import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('wall_following_assigment'), 'resources', 'teleop_ps4.yaml')
    joy_config = LaunchConfiguration('config', default=config_path)
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    
    
   

    #Declare node 
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev':joy_dev}]

    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[{'config_file': joy_config}]
    )
    # teleop_twist_keyboard=Node(
    # package='teleop_twist_keyboard',
    # executable="teleop_twist_keyboard",
    # output='screen',
    # #prefix = 'xterm -e',
    # name='teleop')

    ld = LaunchDescription()
    
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    #ld.add_action(teleop_twist_keyboard)
    return ld