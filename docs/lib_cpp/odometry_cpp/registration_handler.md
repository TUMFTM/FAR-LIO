# Registration Handler

TUM Autonomous Motorsport package for modular handling of point cloud registration algorithms

The handler aligns an incoming frame to the map by iteratively minimizing a per-correspondence
residual. Each implementation builds a linear system (`JTJ`, `JTr`) per correspondence, weighted by
a [robust kernel](robust_kernel.md), which is then solved for a pose update. Correspondences beyond
the current [correspondence threshold](threshold_handler.md) are ignored.

## Solvers

The solver is selected at runtime via the `registration.solver_type` parameter and shared by all
methods:

- **GaussNewton** — solves the normal equations directly each iteration; fast and the default.
- **LevenbergMarquardt** — adds adaptive damping (`registration.damping_factor`,
  `registration.damping_scale`) with a trust-region gain-ratio test, trading iterations for
  robustness on poorly conditioned problems.

## Implemented Registration Methods

1. **ICP** (point-to-point)
    - minimizes the Euclidean distance between each frame point and its closest map point
    (residual `map - T * frame`), based on
    [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Registration.hpp)
    - lightest-weight method; needs no map normals or covariances
    - CUDA variant `CUDA_ICP` mirrors the CPU implementation on the GPU
2. **GICP** (generalized ICP, distribution-to-distribution)
    - weights each residual by a precision matrix derived from the local point-distribution
    covariances of frame and map (Mahalanobis distance), which better exploits planar structure
    - requires a map/point type carrying normals and covariances (`NORMALS`, `COV`) and is therefore
    paired with a larger neighbor count than ICP
    - CUDA variant `CUDA_GICP` mirrors the CPU implementation on the GPU
