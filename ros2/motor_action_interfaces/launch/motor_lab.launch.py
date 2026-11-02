"""Load one motor configuration and start Action/serial bridge plus optional plot.

No motion client is launched. Start a goal explicitly after READY.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    plot = LaunchConfiguration('plot')
    bridge = Node(
        package='motor_action_interfaces', executable='motor_action_server.py',
        name='motor_action_server', output='screen', parameters=[config], respawn=False)
    rpm_plot = Node(
        package='motor_action_interfaces', executable='motor_plot.py', name='motor_rpm_plot', output='screen',
        condition=IfCondition(plot), respawn=False)
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution([
                FindPackageShare('motor_action_interfaces'), 'config', 'motor.yaml']),
            description='Absolute path to the ROS parameter YAML file.'),
        DeclareLaunchArgument('plot', default_value='true', description='Open the live RPM plot.'),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Motor bridge exited; shutting down this launch.'))])),
        bridge,
        rpm_plot,
    ])
