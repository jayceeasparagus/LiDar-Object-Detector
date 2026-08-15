# LiDAR Object Detector

A C++17 ROS 2 obstacle-detection pipeline for 2D `LaserScan` and 3D `PointCloud2` data. It filters and downsamples LiDAR points, groups them into obstacle clusters, generates oriented bounding boxes, and tracks detections with persistent IDs and Kalman-filtered velocity estimates.

![3D LiDAR detection in RViz2](docs/images/3d_lidar_box.png)

## Highlights

- 2D and 3D LiDAR processing
- Voxel-grid downsampling for 3D point clouds
- Spatial-grid Euclidean clustering
- Axis-aligned and oriented bounding boxes with yaw
- 2D and 3D Kalman-filter tracking
- Persistent track IDs and velocity labels
- ROS 2 publishers, subscribers, parameters, TF, and launch files
- Gazebo simulation with a mobile TurtleBot and 3D LiDAR
- RViz2 visualization and Nav2 costmap compatibility
- 47 GoogleTest tests and Release benchmarks

## Pipeline

```text
LaserScan / PointCloud2
        ↓
Filtering and 3D voxel downsampling
        ↓
Spatial-grid clustering
        ↓
Oriented bounding boxes
        ↓
Kalman filtering and track association
        ↓
RViz2 markers and Nav2-compatible obstacle data
```

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
cd ~/projects/LiDar-Object-Detector

cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure

colcon build --base-paths detector_ros --symlink-install
source install/setup.bash
```

## Run the simulations

2D detector with Nav2 costmap visualization:

```bash
ros2 launch lidar_detector_ros custom_world_detector.launch.py
```

3D detector with the mobile 3D-LiDAR TurtleBot:

```bash
ros2 launch lidar_detector_ros custom_world_3d_detector.launch.py
```

## Performance

The spatial-grid implementation avoids comparing every point with every other point. In Release benchmarks it was approximately **8× faster for a typical 2D scan**, **12× faster for a 10,240-point 3D cloud**, and reached **over 20× speedup on larger 2D inputs** while producing the same cluster assignments as the brute-force reference.

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release

./build-release/benchmarks/clustering_benchmark
./build-release/benchmarks/clustering_benchmark_3d
```

Detailed methodology and results are available in [docs/BENCHMARK_RESULTS.md](docs/BENCHMARK_RESULTS.md).

## Conclusion

This project demonstrates an end-to-end autonomous-systems perception pipeline: efficient LiDAR processing in C++, geometric obstacle detection, real-time tracking, ROS 2 integration, simulation, visualization, and Nav2-compatible output. The focus is obstacle geometry and motion tracking rather than semantic object classification.
