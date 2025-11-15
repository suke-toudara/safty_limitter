from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('safety_limiter')
    default_config = os.path.join(pkg_share, 'config', 'safety_limiter_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to config file'
        ),

        DeclareLaunchArgument(
            'cmd_vel_in',
            default_value='/cmd_vel_raw',
            description='Input cmd_vel topic'
        ),

        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel',
            description='Output cmd_vel topic'
        ),

        DeclareLaunchArgument(
            'cloud',
            default_value='/cloud',
            description='Point cloud topic'
        ),

        Node(
            package='safety_limiter',
            executable='safety_limiter_node',
            name='safety_limiter_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('cmd_vel_in', LaunchConfiguration('cmd_vel_in')),
                ('cmd_vel_out', LaunchConfiguration('cmd_vel_out')),
                ('cloud', LaunchConfiguration('cloud')),
            ]
        )
    ])
