# LiDAR Object Detector

A C++17 ROS 2 LiDAR obstacle detector for 2D LaserScan and 3D PointCloud2 data. The project converts sensor data into obstacle clusters, bounding boxes, persistent track IDs, and RViz visualizations.

![3D simulation](docs/images/3d-simulation.png)

## Features

- 2D LaserScan processing and obstacle clustering
- 3D PointCloud2 filtering and voxel downsampling
- Spatial-grid clustering for 2D and 3D point clouds
- Axis-aligned and oriented bounding boxes with yaw
- 2D and 3D Kalman filters for position and velocity estimates
- Persistent obstacle IDs with missed-detection handling
- ROS 2 nodes and launch files
- RViz2 visualization of point clouds, boxes, labels, and velocities
- Gazebo simulation with a mobile TurtleBot and 3D LiDAR
- Nav2 costmap compatibility for processed obstacle points
- GoogleTest coverage and Release performance benchmarks
- Docker build environment

## Processing pipeline

```text
LaserScan / PointCloud2
        ↓
Range and validity filtering
        ↓
Voxel downsampling for 3D data
        ↓
Spatial-grid clustering
        ↓
Bounding-box generation
        ↓
Kalman filtering and track association
        ↓
ROS 2 topics and RViz2 markers
```

## Build

Source ROS 2 Jazzy first:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/projects/LiDar-Object-Detector

cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure

colcon build --base-paths detector_ros --symlink-install
source install/setup.bash
```

The core test suite currently contains 47 tests.

## 2D simulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/LiDar-Object-Detector/install/setup.bash

ros2 launch lidar_detector_ros custom_world_detector.launch.py
```

This launches the custom Gazebo world, TurtleBot 2D LiDAR, detector node, RViz2, TF, and the Nav2 costmap demonstration.

## 3D simulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/LiDar-Object-Detector/install/setup.bash

ros2 launch lidar_detector_ros custom_world_3d_detector.launch.py
```

Useful topics include:

```text
/lidar/points       sensor_msgs/msg/PointCloud2
/filtered_points    sensor_msgs/msg/PointCloud2
/downsampled_points sensor_msgs/msg/PointCloud2
/obstacles_3d       visualization_msgs/msg/MarkerArray
/odom               nav_msgs/msg/Odometry
```

In RViz2, enable the point-cloud displays and the `MarkerArray` display for `/obstacles_3d`. The marker labels identify tracks and show estimated velocity. They are tracking IDs, not semantic classes such as car or person.

## Performance

The optimized implementation uses spatial hashing to limit distance checks to nearby grid cells. The Release benchmark compares it with a brute-force reference implementation using identical points, parameters, and cluster results.

Median results from five Release runs:

| Input | Brute force | Spatial grid | Speedup |
|---:|---:|---:|---:|
| 2D, 360 points | 0.14 ms | 0.03 ms | 4.4× |
| 2D, 640 points | 0.41 ms | 0.05 ms | 7.7× |
| 2D, 2,048 points | 3.99 ms | 0.31 ms | 13.1× |
| 2D, 5,000 points | 24.27 ms | 1.03 ms | 23.4× |
| 3D, 3,100 points | 9.48 ms | 1.09 ms | 8.7× |
| 3D, 5,000 points | 26.27 ms | 2.33 ms | 10.9× |
| 3D, 10,240 points | 104.29 ms | 9.09 ms | 11.9× |

All benchmark comparisons preserved identical cluster assignments. These values are hardware-dependent; rerun the benchmark on the target machine before making a final performance claim.

Run the benchmarks with:

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release

./build-release/benchmarks/clustering_benchmark
./build-release/benchmarks/clustering_benchmark_3d
```

## Configuration

The main 3D parameters are in `detector_ros/config/detector_3d.yaml` and `detector_3d_mobile.yaml`:

- `voxel_size`: 3D downsampling resolution in meters
- `clustering_distance`: maximum point-to-point connection distance
- `min_cluster_size` and `max_cluster_size`: cluster rejection limits
- `min_z` and `max_z`: vertical obstacle filtering limits
- `tracking_association_distance`: maximum distance for matching a detection to a track
- `tracking_max_missed_frames`: frames a track may be undetected internally
- `tracking_process_noise` and `tracking_measurement_noise`: Kalman filter tuning values

The correct values depend on LiDAR resolution, range, environment geometry, and vehicle motion.

## Limitations

- The detector creates geometric clusters and boxes; it does not classify object types.
- A bounding box can include empty space when the visible point cloud is concave or occluded.
- Fixed clustering distance may split sparse distant LiDAR surfaces.
- The current 3D tracker estimates center position and velocity, while box dimensions use the latest measurement.
- Persistent global mapping and SLAM are outside this project’s scope.

## Project scope

This project focuses on real-time LiDAR perception for autonomous systems: efficient point-cloud processing, geometric obstacle detection, tracking, ROS 2 integration, and visualization. Navigation and SLAM systems can consume the outputs but are not reimplemented here.
