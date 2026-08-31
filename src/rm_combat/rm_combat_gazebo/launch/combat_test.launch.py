import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('rm_combat_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    referee_dir = get_package_share_directory('rm_referee')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')
    default_mode = LaunchConfiguration('default_mode')
    projectile_speed = LaunchConfiguration('projectile_speed')
    projectile_radius = LaunchConfiguration('projectile_radius')

    shooter_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'combat_shooter.urdf.xacro'),
        ' robot_id:=red_robot',
        ' projectile_speed:=', projectile_speed,
        ' projectile_radius:=', projectile_radius,
    ])
    target_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'armor_target.urdf.xacro'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('default_mode', default_value='0'),
        DeclareLaunchArgument('projectile_speed', default_value='12.0'),
        DeclareLaunchArgument('projectile_radius', default_value='0.02'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': os.path.join(package_dir, 'worlds', 'combat_test.world')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='red_robot',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': shooter_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='blue_target',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': target_description}],
        ),
        TimerAction(period=1.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'red_robot', '-topic', '/red_robot/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.02',
                ],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'blue_target', '-topic', '/blue_target/robot_description',
                    '-x', '3.0', '-y', '0.0', '-z', '0.0',
                ],
                output='screen',
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(referee_dir, 'launch', 'combat_system.launch.py')),
            launch_arguments={
                'autostart': autostart,
                'default_mode': default_mode,
                'enable_checkpoint': 'false',
                'navigation_adapter': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'combat.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
