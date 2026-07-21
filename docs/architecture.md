# Architecture

FAR-LIO is a CUDA-accelerated LiDAR-inertial odometry framework organized into two main
components (shown below): a **LiDAR Scan Pipeline** that registers each incoming scan against a
local map on the GPU, and a **Sensor Fusion** backend that fuses the registered poses with
high-frequency IMU data. Green modules are GPU-accelerated (CUDA, via Thrust and cuCollections);
blue modules run on the CPU.

<p align="center">
  <img src="assets/architecture.svg" alt="FAR-LIO architecture" width="820">
</p>

## LiDAR Scan Pipeline (GPU)

Registers each incoming scan to an *adaptive, local submap* to derive the current pose. It is
heavily CUDA-accelerated:

- **Preprocessing & undistortion** — motion distortion is removed using the velocity history from
  the Sensor Fusion: a linear regression over the linear and angular velocities yields a continuous
  motion model, and all points are undistorted in parallel and projected to the scan's final
  timestamp.
- **GICP registration** — a sparsity-aware Generalized ICP built on a novel CUDA voxel hashmap
  (`cuVoxelMap`, following the iVox paradigm) for parallel nearest-neighbor search. Each iteration
  computes point covariances, finds correspondences via kNN search with adaptive thresholding for
  outlier rejection (as in KISS-ICP), and solves a robust least-squares problem with a Cauchy
  kernel; in sparse regions it falls back to a point-to-point term (SA-GICP). Iteration stops on
  convergence or a time budget of twice the LiDAR period.
- **Adaptive, local submap** — a rolling local map accumulated from multiple scans. An *adaptive
  submap density* (ASMD) thins points with range, and distant voxels are pruned to bound the map
  size.

## Sensor Fusion (CPU)

Fuses the registered LiDAR poses with the raw IMU stream and produces the final, high-frequency
odometry (pose and velocity):

- **Kinematic Extended Kalman Filter** — a 100 Hz EKF with a purely kinematic motion model (state:
  position, orientation, velocity). IMU acceleration and angular velocity act as control inputs,
  with centripetal and gravitational terms compensated, and IMU biases calibrated during standstill.
  Reference roll/pitch angles are added to the measurement to limit z-drift.
- **Delay compensation** — time-delayed measurements are projected to the current timestep from the
  filter's state history, upsampling them and spreading their correction over several update cycles
  for smooth, low-latency output.

The EKF output is fed back to the scan pipeline as the initial guess and for undistortion, closing
the loop between the two components.
