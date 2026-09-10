# Map Storage Library

TUM Autonomous Motorsport package for modular storage of point cloud map

The map handler stores the point cloud map and answers nearest-neighbor queries for
[registration](registration_handler.md). It also supports incremental updates (adding registered
frames, removing far points) for the online-mapping mode and computes per-point normals/covariances
for GICP.

## Implemented Map Formats

1. **VOXELHASHMAP**
    - hashes points into fixed-size voxels (bounded points per voxel) for O(1) insertion and
    neighbor lookup, based on
    [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp)
    - CPU implementation backed by [robin_map](https://github.com/Tessil/robin-map)

2. **CUDA_VOXELHASHMAP**
    - GPU-resident equivalent using the `cuco::static_map` type of
    [cuCollections](https://github.com/NVIDIA/cuCollections), keeping the map on the device so
    insertion and neighbor search run in parallel without host transfers
