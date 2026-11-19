"""Keyboard teleop (teleop_twist_keyboard) publishing cmd_vel.

It needs a terminal for key presses. From a desktop session use xterm:=true (opens its own window); otherwise run it
directly in a terminal:  ros2 run teleop_twist_keyboard teleop_twist_keyboard   (or ./dev.sh teleop)

  ros2 launch karr_control teleop.launch.py xterm:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = [{'stamped': LaunchConfiguration('stamped'), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    remap = [('cmd_vel', LaunchConfiguration('cmd_vel'))]
    return LaunchDescription([
        DeclareLaunchArgument('xterm', default_value='false', description='Open teleop in its own xterm window'),
        DeclareLaunchArgument('stamped', default_value='false', description='Publish TwistStamped'),
        DeclareLaunchArgument('cmd_vel', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', output='screen',
             prefix='xterm -fa Monospace -fs 11 -geometry 70x22 -T karr_teleop -e', parameters=params,
             remappings=remap, condition=IfCondition(LaunchConfiguration('xterm'))),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', output='screen',
             emulate_tty=True, parameters=params, remappings=remap,
             condition=UnlessCondition(LaunchConfiguration('xterm'))),
    ])
