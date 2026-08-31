import os
import tempfile

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_duel_world(context):
    simulation_dir = get_package_share_directory('pb_rm_simulation')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    source_world = os.path.join(
        simulation_dir, 'world', 'RMUL2024_world', 'RMUL2024_world.world')
    with open(source_world, encoding='utf-8') as stream:
        world_xml = stream.read()

    state_plugin_xml = '''
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>50.0</update_rate>
    </plugin>
'''

    enabled = LaunchConfiguration('contact_damage').perform(context).lower() \
        in ('1', 'true', 'yes', 'on')
    if enabled:
        plugin_xml = '''
    <plugin name="duel_contact_monitor" filename="libduel_contact_plugin.so">
      <red_model>red_robot</red_model>
      <blue_model>blue_robot</blue_model>
      <report_topic>/referee/internal/collision_reports</report_topic>
      <update_rate>50.0</update_rate>
      <min_speed>0.5</min_speed>
      <separation_time>0.3</separation_time>
      <cooldown>1.0</cooldown>
      <horizontal_normal_z>0.7</horizontal_normal_z>
    </plugin>
'''
        state_plugin_xml += plugin_xml

    marker = '</world>'
    position = world_xml.rfind(marker)
    if position < 0:
        raise RuntimeError(f'invalid RMUL world: missing {marker}')
    world_xml = world_xml[:position] + state_plugin_xml + world_xml[position:]

    temporary = tempfile.NamedTemporaryFile(
        mode='w', prefix='rmul_duel_', suffix='.world', delete=False,
        encoding='utf-8')
    with temporary:
        temporary.write(world_xml)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': temporary.name}.items(),
    )]


def generate_launch_description():
    package_dir = get_package_share_directory('rm_combat_gazebo')
    referee_dir = get_package_share_directory('rm_referee')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gui = LaunchConfiguration('gui')
    autostart = LaunchConfiguration('autostart')
    contact_damage = LaunchConfiguration('contact_damage')
    lan_gateway = LaunchConfiguration('lan_gateway')
    lan_bind_host = LaunchConfiguration('lan_bind_host')
    lan_port = LaunchConfiguration('lan_port')
    lan_red_token = LaunchConfiguration('lan_red_token')
    lan_blue_token = LaunchConfiguration('lan_blue_token')
    lan_referee_token = LaunchConfiguration('lan_referee_token')

    red_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'combat_shooter.urdf.xacro'),
        ' robot_id:=red_robot',
        ' opponent_id:=blue_robot',
        ' body_rgba:="0.80 0.05 0.05 1"',
    ])
    blue_description = Command([
        'xacro ', os.path.join(package_dir, 'urdf', 'combat_shooter.urdf.xacro'),
        ' robot_id:=blue_robot',
        ' opponent_id:=red_robot',
        ' body_rgba:="0.05 0.15 0.85 1"',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('contact_damage', default_value='true'),
        DeclareLaunchArgument('lan_gateway', default_value='true'),
        DeclareLaunchArgument('lan_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('lan_port', default_value='8765'),
        DeclareLaunchArgument('lan_red_token', default_value='red-test-2026'),
        DeclareLaunchArgument('lan_blue_token', default_value='blue-test-2026'),
        DeclareLaunchArgument('lan_referee_token', default_value='referee-test-2026'),
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            os.path.join(get_package_prefix('rm_combat_gazebo'), 'lib')),
        OpaqueFunction(function=_start_duel_world),
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
            parameters=[{
                'use_sim_time': True,
                'frame_prefix': 'red_robot/',
                'robot_description': ParameterValue(red_description, value_type=str),
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='blue_robot',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'frame_prefix': 'blue_robot/',
                'robot_description': ParameterValue(blue_description, value_type=str),
            }],
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'red_robot',
                    '-topic', '/red_robot/robot_description',
                    '-x', '4.3', '-y', '3.35', '-z', '1.16', '-Y', '0.0',
                ],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'blue_robot',
                    '-topic', '/blue_robot/robot_description',
                    '-x', '11.3', '-y', '3.35', '-z', '1.16',
                    '-Y', '3.141592653589793',
                ],
                output='screen',
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(referee_dir, 'launch', 'combat_system.launch.py')),
            launch_arguments={
                'autostart': autostart,
                'default_mode': '1',
                'enable_checkpoint': 'false',
                'navigation_adapter': 'false',
                'shooter_model_name': 'red_robot',
                'blue_robot_id': 'blue_robot',
                'blue_model_name': 'blue_robot',
                'lan_gateway': lan_gateway,
                'lan_bind_host': lan_bind_host,
                'lan_port': lan_port,
                'lan_red_token': lan_red_token,
                'lan_blue_token': lan_blue_token,
                'lan_referee_token': lan_referee_token,
            }.items(),
        ),
        Node(
            package='rm_referee',
            executable='combat_cmd_vel_mux',
            namespace='red_robot',
            name='combat_cmd_vel_mux',
            parameters=[{
                'use_sim_time': True,
                'robot_id': 'red_robot',
                'auto_topic': '/red_robot/cmd_vel_auto',
                'manual_topic': '/red_robot/cmd_vel_manual',
                'output_topic': '/red_robot/cmd_vel_chassis',
                'cancel_navigation': False,
            }],
        ),
        Node(
            package='rm_referee',
            executable='combat_cmd_vel_mux',
            namespace='blue_robot',
            name='combat_cmd_vel_mux',
            parameters=[{
                'use_sim_time': True,
                'robot_id': 'blue_robot',
                'auto_topic': '/blue_robot/cmd_vel_auto',
                'manual_topic': '/blue_robot/cmd_vel_manual',
                'output_topic': '/blue_robot/cmd_vel_chassis',
                'cancel_navigation': False,
            }],
        ),
    ])
