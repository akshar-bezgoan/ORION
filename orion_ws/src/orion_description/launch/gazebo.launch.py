import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('orion_description')

    # 1. Robot State Publisher (Xacro -> URDF)
    xacro_file = os.path.join(pkg_share, 'xacro', 'orion.xacro')
    robot_description = Command(['xacro ', xacro_file])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Harmonic (Empty World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn the Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-name', 'orion',
            '-topic', 'robot_description',
            '-z', '0.1' # Spawn slightly in the air
        ],
        output='screen',
    )

    # 4. ROS-GZ Bridge (Crucial for movement and clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    # 5. Controller Spawners (These will fail until we fix the URDF/YAML)
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller'],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        joint_state_spawner,
        mecanum_spawner,
    ])
