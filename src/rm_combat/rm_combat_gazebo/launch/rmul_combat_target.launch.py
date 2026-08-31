import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('rm_combat_gazebo')
    referee_dir = get_package_share_directory('rm_referee')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')
    default_mode = LaunchConfiguration('default_mode')
    autostart = LaunchConfiguration('autostart')
    lan_gateway = LaunchConfiguration('lan_gateway')
    lan_bind_host = LaunchConfiguration('lan_bind_host')
    lan_port = LaunchConfiguration('lan_port')
    lan_red_token = LaunchConfiguration('lan_red_token')
    lan_blue_token = LaunchConfiguration('lan_blue_token')
    lan_referee_token = LaunchConfiguration('lan_referee_token')
    red_zone_x = LaunchConfiguration('red_zone_x')
    red_zone_y = LaunchConfiguration('red_zone_y')
    blue_zone_x = LaunchConfiguration('blue_zone_x')
    blue_zone_y = LaunchConfiguration('blue_zone_y')
    zone_z = LaunchConfiguration('zone_z')
    target_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'armor_target.urdf.xacro'),
    ])
    red_zone_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'spawn_zone.urdf.xacro'),
        ' zone_name:=red_spawn_zone',
        ' rgba:="0.9 0.05 0.05 0.65"',
        ' gazebo_material:=Gazebo/Red',
    ])
    blue_zone_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'spawn_zone.urdf.xacro'),
        ' zone_name:=blue_spawn_zone',
        ' rgba:="0.05 0.15 0.9 0.65"',
        ' gazebo_material:=Gazebo/Blue',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('target_x', default_value='11.3'),
        DeclareLaunchArgument('target_y', default_value='3.35'),
        DeclareLaunchArgument('target_z', default_value='1.10'),
        DeclareLaunchArgument('default_mode', default_value='0'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('lan_gateway', default_value='false'),
        DeclareLaunchArgument('lan_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('lan_port', default_value='8765'),
        DeclareLaunchArgument('lan_red_token', default_value='red-demo'),
        DeclareLaunchArgument('lan_blue_token', default_value='blue-demo'),
        DeclareLaunchArgument('lan_referee_token', default_value='referee-demo'),
        DeclareLaunchArgument('red_zone_x', default_value='4.3'),
        DeclareLaunchArgument('red_zone_y', default_value='3.35'),
        DeclareLaunchArgument('blue_zone_x', default_value='11.3'),
        DeclareLaunchArgument('blue_zone_y', default_value='3.35'),
        DeclareLaunchArgument('zone_z', default_value='1.01'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='blue_target',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': target_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='red_spawn_zone',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': ParameterValue(
                    red_zone_description, value_type=str),
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='blue_spawn_zone',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': ParameterValue(
                    blue_zone_description, value_type=str),
            }],
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'blue_target', '-topic', '/blue_target/robot_description',
                    '-x', target_x, '-y', target_y, '-z', target_z,
                ],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'red_spawn_zone',
                    '-topic', '/red_spawn_zone/robot_description',
                    '-x', red_zone_x, '-y', red_zone_y, '-z', zone_z,
                ],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'blue_spawn_zone',
                    '-topic', '/blue_spawn_zone/robot_description',
                    '-x', blue_zone_x, '-y', blue_zone_y, '-z', zone_z,
                ],
                output='screen',
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(referee_dir, 'launch', 'combat_system.launch.py')),
            launch_arguments={
                'autostart': autostart,
                'enable_checkpoint': 'true',
                'navigation_adapter': 'true',
                'shooter_model_name': 'robot',
                'default_mode': default_mode,
                'lan_gateway': lan_gateway,
                'lan_bind_host': lan_bind_host,
                'lan_port': lan_port,
                'lan_red_token': lan_red_token,
                'lan_blue_token': lan_blue_token,
                'lan_referee_token': lan_referee_token,
            }.items(),
        ),
    ])
