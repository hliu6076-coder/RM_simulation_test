#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    common_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'wait_for_transform': 0.2,
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),

        Node(
            package='rm_slam',
            executable='rgb_depth_node',
            name='rgb_depth_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='rm_slam',
            executable='scan_process_node',
            name='scan_process_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='rm_slam',
            executable='odom_depth_transform_node',
            name='odom_depth_transform_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[common_parameters, {
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'qos_scan': 2,
                'topic_queue_size': 50,
                'sync_queue_size': 50,
                'Reg/Strategy': '1',
                'Grid/3D': 'true',
                'Grid/FromDepth': 'false',
                'Grid/RangeMin': '0.2',
                'Grid/RangeMax': '30.0',
            }],
            remappings=[
                ('odom', '/odom'),
                ('scan_cloud', '/points_base'),
            ],
            output='screen'),

        Node(
            condition=IfCondition(use_rtabmap_viz),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[common_parameters, {
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'qos_scan': 2,
                'topic_queue_size': 50,
                'sync_queue_size': 50,
            }],
            remappings=[
                ('odom', '/odom'),
                ('scan_cloud', '/points_base'),
            ],
            output='screen'),
    ])
