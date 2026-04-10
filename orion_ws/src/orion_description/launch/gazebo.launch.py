import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory('orion_description')
    xacro_file = os.path.join(pkg_path, 'xacro', 'orion.xacro')

    # Process xacro → robot_description string
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # ── Robot State Publisher ──────────────────────────────────────────
    # use_sim_time MUST be true so RSP stamps TF with Gazebo sim time,
    # not wall clock — otherwise RViz2 / Nav2 will reject TF frames.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── Spawn robot into Gazebo ────────────────────────────────────────
    # z = wheel_radius (0.05) + base_z/2 (0.0525) = 0.1025 m above ground.
    # Spawning at z=0.5 causes a drop and bounce — use exact height instead.
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'orion',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.155',   # slight clearance above computed resting height
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # ── Controller spawners ────────────────────────────────────────────
    # Stagger spawners: gz_ros2_control needs ~3 s to initialise after spawn.
    joint_state_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    arm_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    gripper_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # ── ROS ↔ Gazebo topic bridge ──────────────────────────────────────
    # Format: topic@ros_type[gz_type   = GZ → ROS  (unidirectional)
    #         topic@ros_type]gz_type   = ROS → GZ  (unidirectional)
    #         topic@ros_type@gz_type   = bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sim time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Base velocity command  (ROS → GZ)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Odometry  (GZ → ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF from MecanumDrive (GZ → ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # LiDAR scan  (GZ → ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Joint states from gz_ros2_control (GZ → ROS)
            # Note: gz_ros2_control publishes /joint_states natively via
            # JointStateBroadcaster, no bridge needed for that topic.
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
        joint_state_spawner,
        arm_spawner,
        gripper_spawner,
        bridge,
    ])
