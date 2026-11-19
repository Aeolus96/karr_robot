"""Show K.A.R.R. in rviz2 with joint sliders.

  ros2 launch karr_description display.launch.py
  ros2 launch karr_description display.launch.py model:=/path/to/robot.urdf gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('karr_description')
    default_model = os.path.join(pkg_share, 'urdf', 'karr.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # xacro passes plain URDF through unchanged, so this handles .urdf and .xacro
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model,
                              description='Absolute path to a .urdf or .xacro file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='rviz2 config file'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             output='screen'),
    ])
