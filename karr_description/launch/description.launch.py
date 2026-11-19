"""Publish the K.A.R.R. description (robot_state_publisher only). Include this from sim / bringup launches.

  ros2 launch karr_description description.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_model = os.path.join(get_package_share_directory('karr_description'), 'urdf', 'karr.urdf')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model, description='Path to a .urdf or .xacro'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (simulation)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str),
                          'use_sim_time': use_sim_time}]),
    ])
