import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('rm_referee'), 'config', 'combat_rules.yaml')
    bind_host = LaunchConfiguration('bind_host')
    port = LaunchConfiguration('port')
    red_token = LaunchConfiguration('red_token')
    blue_token = LaunchConfiguration('blue_token')
    referee_token = LaunchConfiguration('referee_token')
    state_broadcast_hz = LaunchConfiguration('state_broadcast_hz')

    return LaunchDescription([
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('red_token', default_value='red-demo'),
        DeclareLaunchArgument('blue_token', default_value='blue-demo'),
        DeclareLaunchArgument('referee_token', default_value='referee-demo'),
        DeclareLaunchArgument('state_broadcast_hz', default_value='2.0'),
        Node(
            package='rm_referee',
            executable='referee_node',
            name='referee_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': False, 'autostart': False}],
        ),
        Node(
            package='rm_referee',
            executable='referee_lan_gateway',
            name='referee_lan_gateway',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'bind_host': bind_host,
                'port': ParameterValue(port, value_type=int),
                'red_token': red_token,
                'blue_token': blue_token,
                'referee_token': referee_token,
                'state_broadcast_hz': ParameterValue(
                    state_broadcast_hz, value_type=float),
            }],
        ),
    ])
