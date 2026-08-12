import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    package_share = get_package_share_directory("lidar_detector_ros")

    parameter_file = os.path.join(package_share, "config", "detector.yaml",)

    rviz_config = os.path.join(package_share, "rviz", "detector.rviz",)

    nav2_params = os.path.join(package_share, "config", "nav2_costmap.yaml",)

    return LaunchDescription([Node(package="lidar_detector_ros", executable="synthetic_scan_publisher", name="synthetic_scan_publisher", output="screen",),
    Node(package="lidar_detector_ros", executable="lidar_detector_node",
    name="lidar_detector_node", output="screen", parameters=[parameter_file],),
    Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", rviz_config],),
    Node(package="tf2_ros", executable="static_transform_publisher", name="laser_static_transform", arguments=[
        "--x", "0.0", "--y", "0.0", "--z", "0.2", "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0", "--frame-id", "base_link", "--child-frame-id", "laser",],output="screen",),
    Node(package="nav2_costmap_2d", executable="nav2_costmap_2d", name="costmap", output="screen", parameters=[nav2_params],),
    Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_costmap", output="screen", parameters=[{
        "autostart": True,
        "node_names": ["costmap"],
    }],),
    ])