"""Measured wheel JointState -> robot_state_publisher -> RViz. No motion trigger."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('motor_action_interfaces'))
    config = LaunchConfiguration('config')
    clock = {'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    description = (share/'urdf/single_wheel.urdf').read_text()
    bridge = Node(package='motor_action_interfaces',executable='motor_action_server.py',
                  name='motor_action_server',output='screen',respawn=False,
                  parameters=[config,{'publish_signed_encoder':True}],
                  condition=IfCondition(LaunchConfiguration('bridge')))
    joint = Node(package='motor_action_interfaces',executable='wheel_joint_state.py',
                 name='wheel_joint_state',output='screen',parameters=[config,clock],respawn=False)
    transforms = Node(package='robot_state_publisher',executable='robot_state_publisher',
                      name='robot_state_publisher',output='screen',
                      parameters=[{'robot_description':ParameterValue(description,value_type=str)},clock],respawn=False)
    viewer = Node(package='rviz2',executable='rviz2',name='wheel_rviz',output='screen',
                  arguments=['-d',str(share/'rviz/wheel.rviz')],parameters=[clock],
                  additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},
                  condition=IfCondition(LaunchConfiguration('rviz')),respawn=False)
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/motor.yaml')),
        DeclareLaunchArgument('bridge',default_value='true',description='Start hardware bridge; false for replay.'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[
            EmitEvent(event=Shutdown(reason='Hardware bridge exited.'))])),
        bridge,joint,transforms,viewer,
    ])
