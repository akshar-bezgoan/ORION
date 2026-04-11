import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg           = get_package_share_directory('orion_description')
    xacro_file    = os.path.join(pkg, 'xacro',  'orion.xacro')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', 'empty.sdf'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    spawn = TimerAction(period=2.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'orion',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.1025',
                '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            ],
            output='screen'
        )
    ])

    bridge = TimerAction(period=4.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='orion_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True,
            }],
            output='screen'
        )
    ])

    jsb = TimerAction(period=5.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    wheels = TimerAction(period=6.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_velocity_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    arm = TimerAction(period=7.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    grip = TimerAction(period=8.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    mecanum_ik = TimerAction(period=9.0, actions=[
        Node(
            package='orion_description',
            executable='mecanum_ik_node.py',
            name='mecanum_ik_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge,
        jsb,
        wheels,
        arm,
        grip,
        mecanum_ik,
    ])