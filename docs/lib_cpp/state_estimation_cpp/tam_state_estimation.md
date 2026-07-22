# State Estimation Library

A modular, **header-only** C++ state estimation library, completely free of ROS 2 dependencies. It fuses an arbitrary
number of localization, linear-velocity and IMU sources into a consistent vehicle state with an Extended Kalman Filter
(EKF), following the concepts in this
[paper](https://www.sciencedirect.com/science/article/pii/S2405896319303957).

## Highlights

- **Header-only** — depend on it, include one header, done.
- **Compile-time configurable** — the filter dimension (2D / 3D) and the wheelspeed model are template parameters, so a
  configuration never pays at runtime for a capability it does not use.
- **Any number of sensors** — N localization, N linear-velocity and N IMU inputs.
- **Robust** — per-sensor validity tracking / graceful degradation via a state machine, adaptive measurement covariance,
  and outlier rejection (box or Mahalanobis).
- **ROS-agnostic** — the ROS 2 wrapper is a thin layer in
  [tam_state_estimation_node](../../ros_packages/tam_state_estimation_node.md).

## Design

`StateEstimation<TConfig, VehicleModel>` is the top-level class, templated on:

- **`TConfig`** — the state / measurement / input layout, from
  [state_estimation_constants](state_estimation_constants.md) (`EKF_2D`, `EKF_3D`).
- **`VehicleModel`** — an `enum class` selecting how wheelspeed odometry is fused:
  `Kinematic` (not fused), `NonHolonomic` (fused as linear velocity, `vy = 0`), or
  `SingleTrack` (fused, with a side-slip angle estimated from the steering angle).

It composes the following header-only submodules (under `include/tam_state_estimation/`):

<!-- markdownlint-disable MD013 -->
| Submodule | Responsibility |
|-----------|----------------|
| `submodules/imu_handler.hpp` | Weighted fusion, bias compensation and FIR pre-filtering of the raw IMU inputs |
| `submodules/state_machine.hpp` | Track the validity of every input modality, decide what is fused, expose the overall status |
| `submodules/vehicle_model_handler.hpp` | Turn wheelspeeds into a linear-velocity measurement per the chosen `VehicleModel` |
| `submodules/ref_orientation_handler.hpp` | Estimate a pitch/roll reference orientation from the IMU and predicted odometry |
| `kalman_filter/` | The EKF (`base`, `ekf`, `ekf_2d`, `ekf_3d`): adaptive measurement covariance + outlier rejection |
| `helper/` | Shared building blocks: `fir` (FIR filter), `outlier_rejection`, config/param `types` |
<!-- markdownlint-enable MD013 -->

## 2D state estimation

State vector, fused from N localization sources, N linear-velocity measurements and N IMUs:

$$
x = \left[\begin{array}{c} {X_m} \\ {Y_m} \\ {\psi} \\ {v_x} \\ {v_y} \end{array}\right]
$$

$X_m$, $Y_m$, $\psi$ are the transform from the local Cartesian frame to the vehicle footprint frame; $v_x$, $v_y$ are
the velocity in the vehicle frame. The EKF uses a constant-velocity prediction and applies the IMU signals directly in
the prediction step with the input vector:

$$
u = \left[\begin{array}{c} {\omega_z} \\ {a_x} \\ {a_y} \end{array}\right]
$$

## 3D state estimation

$$
x = \left[\begin{array}{c} {X_m} \\ {Y_m} \\ {Z_m} \\ {\phi} \\ {\theta} \\ {\psi} \\ {v_x} \\ {v_y} \\ {v_z} \end{array}\right]
\qquad
u = \left[\begin{array}{c} {\omega_x} \\ {\omega_y} \\ {\omega_z} \\ {a_x} \\ {a_y} \\ {a_z} \end{array}\right]
$$

The angular velocity is transformed into a frame perpendicular to the local Cartesian frame before the prediction step:

$$
\left[\begin{array}{c} {\dot{\phi}} \\ {\dot{\theta}} \\ {\dot{\psi}} \end{array}\right]=\left[
\begin{array}{c c c}
    {1} & {\sin(\phi)\tan(\theta)} & {\cos(\phi)\tan(\theta)} \\
    {0} & {\cos(\phi)}             & {-\sin(\phi)} \\
    {0} & {\sin(\phi)\sec(\theta)} & {\cos(\phi)\sec(\theta)}
\end{array}\right]\left[\begin{array}{c} {\omega_x} \\ {\omega_y} \\ {\omega_z} \end{array}\right]
$$

The velocity state is likewise transformed from the vehicle frame into that frame to obtain the position prediction.

## State machine

State transitions (leaving `ERROR`/`STALE` requires a software reset):

|                     |  OK  | STALE |  WARN  | STALE | ERROR | STALE | ERROR | STALE |
|---------------------|------|-------|--------|-------|-------|-------|-------|-------|
| any valid_loc_x     |   x  |   x   |    x   |   x   |       |       |       |       |
| any valid_lin_vel_x |   x  |   x   |        |       |   x   |   x   |       |       |
| any valid_imu_x     |   x  |       |    x   |       |   x   |       |   x   |       |

## Usage

```cmake
find_package(tam_state_estimation REQUIRED)
ament_target_dependencies(<your_target> tam_state_estimation)
```

```cpp
#include "tam_state_estimation/state_estimation.hpp"

// 3D EKF with a non-holonomic wheelspeed model
tam::core::state::StateEstimation<tam::core::state::EKF_3D,
  tam::core::state::VehicleModel::NonHolonomic> state_estimation;
```
