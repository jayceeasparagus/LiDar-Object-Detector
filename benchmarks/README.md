# Benchmark methodology

The benchmark executables compare algorithms using identical input data and identical compiler settings.

## Build and run

Always use a Release build when recording final numbers:

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release

./build-release/benchmarks/clustering_benchmark
./build-release/benchmarks/clustering_benchmark_3d
```

Run each executable at least five times with the simulator and other CPU-heavy programs stopped. Report the median result.

## Comparisons

The 2D benchmark compares:

- Unoptimized reference: brute-force Euclidean clustering, approximately O(n^2).
- Optimized implementation: spatial-grid Euclidean clustering, approximately O(n + local neighbor checks) for well-distributed points.

The first 3D section makes the same comparison on identical 3D points. It verifies that both algorithms return the same cluster partition before printing a speedup.

The second 3D section compares complete processing choices:

- Unoptimized pipeline: brute-force clustering on all raw points.
- Optimized pipeline: voxel downsampling followed by spatial-grid clustering.

This pipeline comparison can produce a much larger speedup because the optimized path processes fewer points. Keep it separate from the same-input clustering speedup.

## Representative input sizes

- 2D: 360 and 640 points approximate common LaserScan sizes.
- 3D: 10,240 raw points matches the 16 x 640 simulated LiDAR cloud.
- 3D: 3,100 and 5,000 points approximate filtered/downsampled simulator stages.

The generated data is deterministic, so results are repeatable. It is not a substitute for a recorded real or simulator point cloud. For a resume, use the same-input spatial-grid speedup as the primary algorithm result. Treat the synthetic end-to-end pipeline speedup as a development benchmark until it is confirmed with recorded sensor data.

## Speedup calculation

```text
speedup = unoptimized_time_ms / optimized_time_ms
```

Example: 12 ms / 1 ms = 12x faster.

Do not compare a Debug brute-force build against a Release spatial-grid build. Both algorithms must use the same Release binary and the same input points.
