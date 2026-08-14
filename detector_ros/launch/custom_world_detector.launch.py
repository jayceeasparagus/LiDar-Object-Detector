import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detector_share = get_package_share_directory("lidar_detector_ros")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")

    turtlebot_launch_directory = os.path.join(turtlebot_share, "launch")
    world_file = os.path.join(
        detector_share,
        "worlds",
        "obstacle_test_world.sdf",
    )
    detector_parameters = os.path.join(
        detector_share,
        "config",
        "detector.yaml",
    )
    rviz_config = os.path.join(
        detector_share,
        "rviz",
        "detector.rviz",
    )
    nav2_parameters = os.path.join(
        detector_share,
        "config",
        "nav2_costmap_turtlebot.yaml",
    )

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

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_launch_directory,
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_launch_directory,
                "spawn_turtlebot3.launch.py",
            )
        ),
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(turtlebot_share, "models"),
            ),
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_turtlebot,
            Node(
                package="lidar_detector_ros",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="lidar_detector_ros",
                executable="lidar_detector_node",
                name="lidar_detector_node",
                output="screen",
                parameters=[
                    detector_parameters,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config, "-f", "odom"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="nav2_costmap_2d",
                        executable="nav2_costmap_2d",
                        name="costmap",
                        output="screen",
                        parameters=[
                            nav2_parameters,
                            {"use_sim_time": use_sim_time},
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_costmap",
                        output="screen",
                        parameters=[
                            {
                                "autostart": True,
                                "node_names": ["costmap"],
                                "bond_timeout": 0.0,
                                "use_sim_time": use_sim_time,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
