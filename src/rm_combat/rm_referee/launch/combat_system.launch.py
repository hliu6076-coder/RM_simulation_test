import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rm_referee'), 'config', 'combat_rules.yaml')
    autostart = LaunchConfiguration('autostart')
    enable_checkpoint = LaunchConfiguration('enable_checkpoint')
    navigation_adapter = LaunchConfiguration('navigation_adapter')
    shooter_model_name = LaunchConfiguration('shooter_model_name')
    blue_robot_id = LaunchConfiguration('blue_robot_id')
    blue_model_name = LaunchConfiguration('blue_model_name')
    default_mode = LaunchConfiguration('default_mode')
    lan_gateway = LaunchConfiguration('lan_gateway')
    lan_bind_host = LaunchConfiguration('lan_bind_host')
    lan_port = LaunchConfiguration('lan_port')
    lan_red_token = LaunchConfiguration('lan_red_token')
    lan_blue_token = LaunchConfiguration('lan_blue_token')
    lan_referee_token = LaunchConfiguration('lan_referee_token')
    lan_state_broadcast_hz = LaunchConfiguration('lan_state_broadcast_hz')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_checkpoint', default_value='false'),
        DeclareLaunchArgument('navigation_adapter', default_value='false'),
        DeclareLaunchArgument('shooter_model_name', default_value='red_robot'),
        DeclareLaunchArgument('blue_robot_id', default_value='blue_target'),
        DeclareLaunchArgument('blue_model_name', default_value='blue_target'),
        DeclareLaunchArgument('default_mode', default_value='0'),
        DeclareLaunchArgument('lan_gateway', default_value='false'),
        DeclareLaunchArgument('lan_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('lan_port', default_value='8765'),
        DeclareLaunchArgument('lan_red_token', default_value='red-demo'),
        DeclareLaunchArgument('lan_blue_token', default_value='blue-demo'),
        DeclareLaunchArgument('lan_referee_token', default_value='referee-demo'),
        DeclareLaunchArgument('lan_state_broadcast_hz', default_value='2.0'),
        Node(
            package='rm_referee',
            executable='referee_node',
            name='referee_node',
            output='screen',
            parameters=[default_params, {
                'autostart': autostart,
                'enable_checkpoint': enable_checkpoint,
                'red_robot_id': 'red_robot',
                'blue_robot_id': blue_robot_id,
                'red_model_name': shooter_model_name,
                'blue_model_name': blue_model_name,
                'default_mode': ParameterValue(default_mode, value_type=int),
            }],
        ),
        Node(
            package='rm_referee',
            executable='navigation_adapter',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_id': 'red_robot'}],
            condition=IfCondition(navigation_adapter),
        ),
        Node(
            package='rm_referee',
            executable='referee_lan_gateway',
            name='referee_lan_gateway',
            output='screen',
            parameters=[{
                # LAN control must remain responsive before /clock is available.
                'use_sim_time': False,
                'bind_host': lan_bind_host,
                'port': ParameterValue(lan_port, value_type=int),
                'red_token': lan_red_token,
                'blue_token': lan_blue_token,
                'referee_token': lan_referee_token,
                'state_broadcast_hz': ParameterValue(
                    lan_state_broadcast_hz, value_type=float),
            }],
            condition=IfCondition(lan_gateway),
        ),
    ])
