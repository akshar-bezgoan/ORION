import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_orion     = get_package_share_directory("orion_description")
    pkg_ros_gz    = get_package_share_directory("ros_gz_sim")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="orion",
        description="Gazebo world name without .sdf extension (e.g. orion, maze)",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1025")  # wheel_radius(0.05) + base_z/2(0.0525)

    use_sim_time = LaunchConfiguration("use_sim_time")
    world        = LaunchConfiguration("world")

    xacro_file = os.path.join(pkg_orion, "xacro", "orion.xacro")
    controllers_file = os.path.join(pkg_orion, "config", "ros2_controllers.yaml")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "controllers_file:=",
            controllers_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # World SDF resolved from the "world" launch argument at runtime
    from launch.substitutions import PathJoinSubstitution as PJS
    world_sdf_sub = PJS([
        FindPackageShare("orion_description"),
        "worlds",
        [LaunchConfiguration("world"), ".sdf"],
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_sdf_sub],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_orion",
        output="screen",
        arguments=[
            "-name",  "orion",
            "-topic", "robot_description",
            "-x",     LaunchConfiguration("x"),
            "-y",     LaunchConfiguration("y"),
            "-z",     LaunchConfiguration("z"),
        ],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Pickup object pose: gz Pose_V -> ROS 2 geometry_msgs/PoseArray
            "/model/pickup_object/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="mecanum_controller_spawner",
        output="screen",
        arguments=[
            "mecanum_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="gripper_controller_spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_effort_controller_spawner",
        output="screen",
        arguments=[
            "arm_effort_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--inactive", 
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gz_start = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[gazebo],
        )
    )

    robot_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                TimerAction(period=5.0, actions=[spawn_robot]), 
            ],
        )
    )

    bridge_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                TimerAction(period=3.0, actions=[ros_gz_bridge]), 
            ],
        )
    )

    jsb_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    mecanum_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_controller_spawner],
        )
    )

    arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=mecanum_controller_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    effort_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[arm_effort_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            world_arg,
            x_arg,
            y_arg,
            z_arg,

            robot_state_publisher,

            gz_start,
            robot_spawner,
            bridge_spawner,
            jsb_spawner,
            mecanum_controller,
            arm_controller,
            gripper_controller,
            effort_controller,
        ]
    )