# Benchmark results

The benchmark was compiled with CMake and Ninja using `CMAKE_BUILD_TYPE=Release`. Each executable was run five times. The values below are representative medians from those runs.

## Same-input clustering

The brute-force and spatial-grid algorithms receive the same generated points and parameters. The benchmark also compares their complete cluster partitions, not only the number of clusters.

| Dimension | Points | Brute force | Spatial grid | Speedup |
|---|---:|---:|---:|---:|
| 2D | 640 | 0.41 ms | 0.05 ms | 7.7× |
| 2D | 2,048 | 3.99 ms | 0.31 ms | 13.1× |
| 3D | 3,100 | 9.48 ms | 1.09 ms | 8.7× |
| 3D | 5,000 | 26.27 ms | 2.33 ms | 10.9× |
| 3D | 10,240 | 104.29 ms | 9.09 ms | 11.9× |

Every comparison returned `correct=yes`.

## Interpretation

The brute-force reference checks all points while searching for neighbors and has quadratic worst-case behavior. The spatial-grid implementation limits checks to the current cell and neighboring cells. Its practical scaling depends on point density and cell occupancy, so speedup increases with input size but is not a single constant.

The 10,240-point case is representative of the simulated 16-layer by 640-sample 3D LiDAR cloud. The 11.9× result is the primary optimization result because both algorithms processed the same points.

## Reproduction

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release
./build-release/benchmarks/clustering_benchmark
./build-release/benchmarks/clustering_benchmark_3d
```

Results vary with processor, compiler, operating-system load, and data distribution. Final measurements should be repeated on the machine used for deployment.
