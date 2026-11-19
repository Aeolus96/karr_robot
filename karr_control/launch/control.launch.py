"""Start the ros2_control controllers and the karr_drive node. Needs a running controller_manager
(Gazebo's gz_ros2_control, or the real robot's ros2_control_node).

  ros2 launch karr_control control.launch.py use_sim_time:=true steering_mode:=four_wheel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cm = LaunchConfiguration('controller_manager')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', cm, '--controller-manager-timeout', '120'],
                    parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (simulation)'),
        DeclareLaunchArgument('steering_mode', default_value='front',
                              description='front (rear locked) | four_wheel (4WS / double Ackermann)'),
        DeclareLaunchArgument('drive_params',
                              default_value=PathJoinSubstitution([FindPackageShare('karr_control'), 'config', 'drive.yaml']),
                              description='karr_drive parameter file'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),

        spawner('joint_state_broadcaster'),
        spawner('steering_controller'),
        spawner('wheel_controller'),
        Node(package='karr_control', executable='drive_node', name='karr_drive', output='screen',
             parameters=[LaunchConfiguration('drive_params'),
                         {'use_sim_time': use_sim_time, 'steering_mode': LaunchConfiguration('steering_mode')}]),
    ])
