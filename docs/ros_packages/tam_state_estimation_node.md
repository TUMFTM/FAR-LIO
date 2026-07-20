# tam_state_estimation_node

ROS 2 wrapper around the pure-C++ [tam_state_estimation](../lib_cpp/tam_state_estimation.md) library. It subscribes to
the sensor streams published by the
[vehicle interface](https://gitlab.lrz.de/iac/mod_vehicle_interface/-/tree/develop/sensor_interfaces?ref_type=heads),
delay-compensates and converts them, feeds them into the (ROS-free) estimator, and republishes the fused vehicle state
under `/core/state`.

Everything ROS-specific lives in this package — message conversion, delay compensation, the topic watchdog, TF
broadcasting and the node monitor — while every line of estimation logic stays in the library. The node itself is a
class template `stateEstimationNode<TConfig, VehicleModel>`
([node.hpp](https://gitlab.lrz.de/iac/mod_state_estimation/-/blob/develop/ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/node.hpp?ref_type=heads));
each shipped variant is one instantiation of it.

![High level state estimation concept](../images/State_Estimation.png)

## Variants

The node is a composable `rclcpp` component. One variant is registered per *(filter dimension × wheelspeed model)*;
each is available both as a loadable component (plugin) and as a standalone executable. Pick the one matching your
vehicle and sensor setup.

<!-- markdownlint-disable MD013 -->
| Component (plugin) | Executable | Filter | Wheelspeed model |
|--------------------|------------|--------|------------------|
| `tam_state_estimation_node::StateEstimation2DEKFKin` | `tam_state_estimation_node_ekf_2d_kin` | 2D EKF | Kinematic |
| `tam_state_estimation_node::StateEstimation2DEKFNh`  | `tam_state_estimation_node_ekf_2d_nh`  | 2D EKF | Non-holonomic |
| `tam_state_estimation_node::StateEstimation3DEKFKin` | `tam_state_estimation_node_ekf_3d_kin` | 3D EKF | Kinematic |
| `tam_state_estimation_node::StateEstimation3DEKFNh`  | `tam_state_estimation_node_ekf_3d_nh`  | 3D EKF | Non-holonomic |
| `tam_state_estimation_node::StateEstimation3DEKFStm` | `tam_state_estimation_node_ekf_3d_stm` | 3D EKF | Single-track |
<!-- markdownlint-enable MD013 -->

*Kinematic* fuses no wheelspeed odometry; *non-holonomic* fuses it as a linear-velocity measurement (`vy = 0`);
*single-track* additionally estimates a side-slip angle from the steering angle.

## Topics

All topics live under the `/core/state` namespace.

### Published

<!-- markdownlint-disable MD013 -->
| Topic | Type | Description |
|-------|------|-------------|
| `/core/state/odometry` | `nav_msgs/msg/Odometry` | Fused pose + velocity |
| `/core/state/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | Fused acceleration |
| `/core/state/s` | `std_msgs/msg/Float64` | Arc length along the raceline |
| `/core/state/s/centerline` | `std_msgs/msg/Float64` | Arc length along the centerline |
| *(TF)* | `geometry_msgs/msg/TransformStamped` | `local_cartesian → vehicle_velocity_cg`, `vehicle_cg → vehicle_velocity_cg` |
<!-- markdownlint-enable MD013 -->

### Subscribed

- **Localization / linear-velocity / IMU inputs** — not hard-coded; the set of sensor topics is built at start-up from
  the ROS parameter overrides (`measurements.*` and `inputs.imus.*`) by `parse_sensor_configs`
  ([helper.hpp](https://gitlab.lrz.de/iac/mod_state_estimation/-/blob/develop/ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/helper.hpp?ref_type=heads)).
- `/vehicle/sensor/wheelspeed_radps`, `/vehicle/sensor/wheelspeed_status` — non-kinematic models only.
- `/vehicle/sensor/steering_report` — single-track model only.

## Configuration

Each sensor is described declaratively via parameters, e.g.:

```yaml
measurements:
  <name>:
    message: { topic: ..., type: ..., timeout_ms: 500 }
    status:  { topic: ..., timeout_ms: 500 }          # optional
    position_m:          { R_init: [...], outlier_bounds: [...] }   # any subset of
    orientation_rad:     { R_init: [...], outlier_bounds: [...] }   # these fusion
    linear_velocity_mps: { R_init: [...], outlier_bounds: [...] }   # groups
inputs:
  imus:
    <name>:
      message: { topic: ..., type: ... }
      filter_coefficients: [...]   # FIR pre-filter (parameter key: imu<N>_filter_coefficients)
      backup: false
```

## Running

Standalone executable:

```bash
ros2 run tam_state_estimation_node tam_state_estimation_node_ekf_3d_nh \
  --ros-args --params-file <your_config>.yml
```

Or load the component into a running container:

```bash
ros2 component load /<container> tam_state_estimation_node \
  tam_state_estimation_node::StateEstimation3DEKFNh \
  --ros-args --params-file <your_config>.yml
```
