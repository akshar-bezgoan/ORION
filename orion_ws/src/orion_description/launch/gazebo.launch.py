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

    orion_description     = get_package_share_directory("orion_description")
    ros_gz_sim    = get_package_share_directory("ros_gz_sim")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )
    world_name = DeclareLaunchArgument(
        "world",
        default_value="orion",
        description="Gazebo world name without .sdf extension (e.g. orion, maze)",
    )
    x = DeclareLaunchArgument("x", default_value="0.0")
    y = DeclareLaunchArgument("y", default_value="0.0")
    z = DeclareLaunchArgument("z", default_value="0.090")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    xacrofile = os.path.join(orion_description, "xacro", "orion.xacro")
    controllersfile = os.path.join(orion_description, "config", "ros2_controllers.yaml")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacrofile,
            " ",
            "controllers_file:=",
            controllersfile,
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

    world_sdf_sub = PathJoinSubstitution([
        FindPackageShare("orion_description"),
        "worlds",
        [LaunchConfiguration("world"), ".sdf"],
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gzs": ["-r -v4 ", world_sdf_sub],
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

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="wheel_velocity_controller_spawner",
        output="screen",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mecanum_drive = Node(
        package="mecanum_drive",
        executable="mecanum_drive_node",
        name="mecanum_drive_node",
        output="screen",
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
            on_exit=[wheel_velocity_controller_spawner],
        )
    )

    arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=wheel_velocity_controller_spawner,
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
            world_name,
            x,
            y,
            z,

            robot_state_publisher,
            mecanum_drive,

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
