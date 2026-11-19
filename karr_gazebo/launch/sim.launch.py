"""K.A.R.R. simulation, everything in one go: Gazebo world, robot + sensors, controllers + drive, rviz, teleop.

  ros2 launch karr_gazebo sim.launch.py
  ros2 launch karr_gazebo sim.launch.py steering_mode:=four_wheel gps_mode:=single cameras:=false
  ros2 launch karr_gazebo sim.launch.py gui:=false rviz:=false teleop:=false     # headless

Arguments are passed through to world.launch.py, spawn.launch.py, karr_control control.launch.py and
teleop.launch.py; each of those can also be launched on its own.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, name, **args):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    cond = args.pop('condition', None)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items(),
                                    condition=cond)


def generate_launch_description():
    L = LaunchConfiguration
    share = get_package_share_directory('karr_gazebo')
    args = [
        ('world', 'basic.sdf', 'World: a name in karr_gazebo/worlds or a full path'),
        ('gui', 'true', 'Gazebo window'),
        ('rviz', 'true', 'rviz2 with robot, TF, lidar and center camera'),
        ('teleop', 'true', 'Keyboard teleop in its own xterm window'),
        ('steering_mode', 'front', 'front (rear locked) | four_wheel (4WS / double Ackermann)'),
        ('gps_mode', 'dual', 'dual (left + right antennas) | single (left)'),
        ('cameras', 'true', 'Simulate the 3 ZED X One S cameras (slow with software rendering)'),
        ('lidar', 'true', 'Simulate the Livox Mid-360 and its IMU'),
        ('x', '0.0', 'Spawn x (m)'), ('y', '0.0', 'Spawn y (m)'), ('yaw', '0.0', 'Spawn yaw (rad)'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args],

        include('karr_gazebo', 'world.launch.py', world=L('world'), gui=L('gui')),
        include('karr_gazebo', 'spawn.launch.py', gps_mode=L('gps_mode'), cameras=L('cameras'), lidar=L('lidar'),
                x=L('x'), y=L('y'), yaw=L('yaw')),
        include('karr_control', 'control.launch.py', use_sim_time='true', steering_mode=L('steering_mode')),
        include('karr_control', 'teleop.launch.py', xterm='true', use_sim_time='true',
                condition=IfCondition(L('teleop'))),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(L('rviz'))),
    ])
