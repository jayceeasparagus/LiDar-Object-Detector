import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detector_share = get_package_share_directory("lidar_detector_ros")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    turtlebot_gazebo_share = get_package_share_directory("turtlebot3_gazebo")

    world_file = os.path.join(
        detector_share,
        "worlds",
        "obstacle_test_world.sdf",
    )
    robot_sdf = os.path.join(
        detector_share,
        "models",
        "turtlebot3_burger_3d",
        "model.sdf",
    )
    bridge_config = os.path.join(
        detector_share,
        "config",
        "turtlebot3_burger_3d_bridge.yaml",
    )
    detector_config = os.path.join(
        detector_share,
        "config",
        "detector_3d_mobile.yaml",
    )
    rviz_config = os.path.join(
        detector_share,
        "rviz",
        "detector_3d.rviz",
    )
    robot_urdf = os.path.join(
        turtlebot_gazebo_share,
        "urdf",
        "turtlebot3_burger.urdf",
    )

    with open(robot_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_turtlebot3_burger_3d",
        output="screen",
        arguments=[
            "-name",
            "burger_3d",
            "-file",
            robot_sdf,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="turtlebot3_burger_3d_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_config}",
        ],
    )

    detector = Node(
        package="lidar_detector_ros",
        executable="pointcloud_inspector_node",
        name="pointcloud_inspector_node",
        output="screen",
        parameters=[detector_config, {"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_3d_mobile",
        output="screen",
        arguments=["-d", rviz_config, "-f", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(detector_share, "models"),
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(turtlebot_gazebo_share, "models"),
            ),
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            bridge,
            detector,
            rviz,
        ]
    )
