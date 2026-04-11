import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('orion_description')
    xacro = os.path.join(pkg, 'xacro', 'orion.xacro')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    robot_description = ParameterValue(Command(['xacro ', xacro]), value_type=str)

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', 'empty.sdf'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='Spawning orion...'),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'orion', '-topic', 'robot_description',
                           '-z', '0.103', '-x', '0', '-y', '0'],
                output='screen'
            )
        ]
    )

    bridge = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg='Starting ros_gz_bridge...'),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='orion_bridge',
                parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
                output='screen',
                respawn=False
            )
        ]
    )

    return LaunchDescription([gz, rsp, spawn, bridge])