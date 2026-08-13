import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    detector_share = get_package_share_directory("lidar_detector_ros")
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")

    turtlebot_launch = os.path.join(
        turtlebot_share,
        "launch",
        "turtlebot3_world.launch.py",
    )
    parameter_file = os.path.join(
        detector_share,
        "config",
        "detector.yaml",
    )
    rviz_config = os.path.join(
        detector_share,
        "rviz",
        "detector.rviz",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot_launch),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="lidar_detector_ros",
                executable="lidar_detector_node",
                name="lidar_detector_node",
                output="screen",
                parameters=[
                    parameter_file,
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
