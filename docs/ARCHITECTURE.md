# Architecture

## Core library

The `detector_core` library is independent of ROS 2. It contains the geometry and algorithms used by both the tests and ROS nodes.

```text
Point2D / Point3D
        ↓
Filtering and conversion
        ↓
Spatial-grid clustering
        ↓
Axis-aligned or oriented bounding boxes
        ↓
Kalman filter and multi-object tracker
```

The 3D path uses a hashed voxel map for downsampling and a hashed spatial grid for neighborhood lookup. The grid avoids comparing every point with every other point when nearby-cell occupancy is limited.

## ROS 2 layer

The ROS 2 package adapts messages and publishes visualization outputs:

```text
sensor_msgs/msg/LaserScan
        ↓
2D detector node
        ↓
MarkerArray / PointCloud2 / Nav2 obstacle input
```

```text
sensor_msgs/msg/PointCloud2
        ↓
3D point-cloud inspector node
        ↓
filtered and downsampled PointCloud2
        ↓
3D clusters and bounding boxes
        ↓
tracked MarkerArray with IDs and velocity
```

## Tracking

Each detection is represented by its bounding-box center. The Kalman filter predicts the next center position and velocity. The multi-object tracker associates detections with the nearest predicted track within the configured distance. Unmatched detections create new IDs; unmatched tracks are retained internally for a limited number of frames and then removed.

The tracker labels obstacles with persistent numeric IDs. These IDs represent temporal identity, not semantic object classes.
