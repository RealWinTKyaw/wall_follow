from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare('husky_control'),
        'config',
        'localization.yaml'],
    )

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('wall_following_assigment'), 'resources', 'twist_mux.yaml']
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/husky_velocity_controller/cmd_vel_unstamped')},
        parameters=[filepath_config_twist_mux]
    )
    ld.add_action(node_twist_mux)

    # node_ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_node',
    #     output='screen',
    #     parameters=[config_husky_ekf],
    #     )
    # ld.add_action(node_ekf)

    primary_imu_enable = EnvironmentVariable('CPR_IMU', default_value='false')

    if (primary_imu_enable.perform(lc)) == 'true':
        config_imu_filter = PathJoinSubstitution(
            [FindPackageShare('husky_control'),
            'config',
            'imu_filter.yaml'],
        )
        node_imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[config_imu_filter]
        )
        ld.add_action(node_imu_filter)

    return ld