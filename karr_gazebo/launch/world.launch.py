"""Start Gazebo (Harmonic) with a world. Makes karr_description meshes findable (package:// URIs).

  ros2 launch karr_gazebo world.launch.py
  ros2 launch karr_gazebo world.launch.py world:=/path/to/my_world.sdf gui:=false
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world) and not os.path.exists(world):
        world = os.path.join(get_package_share_directory('karr_gazebo'), 'worlds', world)
    gz_args = f'-r -v {LaunchConfiguration("verbosity").perform(context)} {world}'
    if LaunchConfiguration('gui').perform(context).lower() not in ('true', '1'):
        gz_args += ' -s'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='basic.sdf',
                              description='World file: a name in karr_gazebo/worlds or a full path'),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window (false = server only)'),
        DeclareLaunchArgument('verbosity', default_value='2', description='Gazebo log level 0-4'),
        # package://karr_description/... -> <prefix>/share/karr_description/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(get_package_prefix('karr_description'), 'share')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(get_package_share_directory('karr_gazebo'), 'worlds')),
        OpaqueFunction(function=_gazebo),
    ])
