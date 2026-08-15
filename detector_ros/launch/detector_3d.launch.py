import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lidar_detector_ros")

    parameter_file = os.path.join(
        package_share,
        "config",
        "detector_3d.yaml",
    )

    rviz_config = os.path.join(
        package_share,
        "rviz",
        "detector_3d.rviz",
    )

    return LaunchDescription([
        Node(
            package="lidar_detector_ros",
            executable="pointcloud_inspector_node",
            name="pointcloud_inspector_node",
            output="screen",
            parameters=[parameter_file],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_3d",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])