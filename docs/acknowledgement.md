# Acknowledgements

FAR-LIO stands on the shoulders of excellent open-source repositories and libraries. If
FAR-LIO is useful to you, please consider giving these projects a ⭐ as well — they made it
possible.

## Algorithms

- [KISS-ICP](https://github.com/PRBonn/kiss-icp) — the inspiration for the general architecture of the odometry package and source of the adaptive-thresholding scheme.
- [fast_gicp](https://github.com/koide3/fast_gicp) — the basis for covariance regularization and factor-based architecture of FAR-LIO.

## Libraries

- [cuCollections](https://github.com/NVIDIA/cuCollections) — CUDA hash map behind `cuVoxelMap`.
- [Sophus](https://github.com/strasdat/Sophus) — Lie-group types for SE(3)/SO(3).
- [oneTBB](https://github.com/uxlfoundation/oneTBB) — parallelism on the CPU side.
- [robin-map](https://github.com/Tessil/robin-map) — fast hash map.
- [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) — point-cloud types and I/O.
- [Eigen](https://gitlab.com/libeigen/eigen) — linear algebra.
- [GoogleTest](https://github.com/google/googletest) — unit testing.
- [rerun](https://github.com/rerun-io/rerun) — optional visualization.
- [tsl](https://github.com/TUMFTM/tsl) — time-series logging and the debug messages FAR-LIO emits.
- [TAM__param_management](https://github.com/TUMFTM/TAM__param_management) - parameter management.
