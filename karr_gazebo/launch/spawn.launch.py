"""Put K.A.R.R. into a running Gazebo: robot_state_publisher (sim xacro), spawn, and ROS <-> Gazebo bridges.

  ros2 launch karr_gazebo spawn.launch.py gps_mode:=single cameras:=false x:=2.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('karr_gazebo')
    cfg = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    L = LaunchConfiguration
    sim_time = {'use_sim_time': True}

    def bridge(name, config, condition=None):
        return Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'bridge_{name}', output='screen',
                    parameters=[{'config_file': cfg(config), **sim_time}], condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('gps_mode', default_value='dual', description='dual (left + right antennas) | single (left)'),
        DeclareLaunchArgument('cameras', default_value='true', description='Simulate the 3 ZED cameras'),
        DeclareLaunchArgument('lidar', default_value='true', description='Simulate the Livox Mid-360 (+ IMU)'),
        DeclareLaunchArgument('name', default_value='karr', description='Gazebo model name'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('karr_description'), 'launch', 'description.launch.py')),
            launch_arguments={
                'model': os.path.join(share, 'urdf', 'karr_sim.urdf.xacro'),
                'xacro_args': ['gps_mode:=', L('gps_mode'), ' cameras:=', L('cameras'), ' lidar:=', L('lidar')],
                'use_sim_time': 'true',
            }.items()),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', L('name'),
                        '-x', L('x'), '-y', L('y'), '-z', L('z'), '-Y', L('yaw')]),

        bridge('base', 'bridge_base.yaml'),
        bridge('lidar', 'bridge_lidar.yaml', IfCondition(L('lidar'))),
        bridge('cameras', 'bridge_cameras.yaml', IfCondition(L('cameras'))),
        bridge('gps_left', 'bridge_gps_left.yaml'),
        bridge('gps_right', 'bridge_gps_right.yaml', IfCondition(EqualsSubstitution(L('gps_mode'), 'dual'))),
    ])
