# Voxel Tools Library

TUM Autonomous Motorsport package for voxelization tools

Header-only helpers shared by the [map handler](map_handler.md) and
[preprocessing handler](preprocessing_handler.md) for mapping points to voxels and reducing point
density. Each function has a CPU and a CUDA implementation.

## Functionalities

1. **Voxel Downsampling** (`voxel_downsample`)
    - keeps one representative point per voxel of a given size, based on
    [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelUtils.hpp)
2. **Double Voxel Downsampling** (`voxel_doubledownsample`)
    - downsamples twice with different voxel sizes (a fine pass at `0.5 x` followed by a coarse
    pass at `1.5 x`), yielding a more uniform spatial distribution than a single pass
3. **Voxel Utilities**
    - `point_to_voxel` (point → voxel index) and `get_adjacent_voxels` (neighbor voxel indices),
    used by the map handler for insertion and neighbor search
