# tam_odometry

ROS2 package providing the ROS2 API for the [Odometry Library](../lib_cpp/odometry_cpp.md).
Due to its templated structure, the `tam_odometry` package provides the source for all scan registration. It also supports CUDA implementations for several modules/functions.

## Available Modes

You can choose, whether you want to run the node as a standalone (internal prediction model) or
in combination with an external filter such as an EKF (external prediction model).
Note that this needs to be configured at compile time, so you have to choose an appropriate
executable.

<!-- markdownlint-disable MD013 -->
| Prediction Model | Description |
| -------------------------------- | ----------- |
| external (`EXT`) | Initial guess for registration taken from an external state estimator (e.g. an EKF). In that case, the node subscribes to `/tf` (where the estimator publishes it) and takes the corresponding transformation as an initial guess. |
| Internal (e.g. `CV`) | Standalone usage (e.g. with only LiDAR data as an input) with a different prediction model (e.g. constant velocity). |
<!-- markdownlint-enable MD013 -->

Furthermore, you can use each executable without ("odometry-mode", usage for map creation) or with a prior map
("localization-mode"). This may be configured via a config parameter `pipeline.update_map`.

<!-- markdownlint-disable MD013 -->
| `update_map` | Description |
| -------------------------------- | ----------- |
| `true` | The node builds an online map with a limited range around its current positions (`map.max_distance`) and registers all incoming frames to the current online map. Note that in this case, your odometry will always start at position `0` as no global coordinates are involved. |
| `false` | The node loads a prior pointcloud map from a file (`node.input_map`) and registers incoming frames to this map  (no modification of the map). As the map is georeferenced, you need to have an appropriate initial guess for the registration. However, the registered pose is also in a global coordinate frame. |
<!-- markdownlint-enable MD013 -->

## Available ROS2 components

<!-- markdownlint-disable MD013 -->
| Executable | Description |
| -------------------------------- | ----------- |
| `tam_icp_ext` | (CPU-based ICP with external initial guess - from `/tf`) |
| `tam_icp_cv` | (CPU-based ICP with constant-velocity model) |
| `tam_radar_icp_ext` | (CPU-based ICP leverage radar-specific information and an external initial guess - from `/tf`) |
| `tam_gicp_ext` | (CPU-based GICP with external initial guess - from `/tf`) |
| `tam_cuda_icp_ext` | (GPU-based ICP with external initial guess - from `/tf`) |
| `tam_cuda_gicp_ext` | (GPU-based ICP with external initial guess - from `/tf`) |

## Interfaces

### Prior Maps

A prior pointcloud map can be provided to the node in two ways: via a ROS2 service (see
[ROS Topics/Services](#ros-topicsservices)), or, for simple debugging, by pointing `node.input_map`
to a `.pcd` file (given `pipeline.update_map = false`). The map is expected to be georeferenced, so
that the registered poses are expressed in a global coordinate frame; in that case you also need an
appropriate initial guess for the registration.

### ROS Topics/Services

Note: Apart from `tf_static` and `/tf`, all topics are configurable via the config.
<!-- markdownlint-disable MD013 -->
| Subscribed Topics (Message Type) | Description |
| -------------------------------- | ----------- |
| `/tf_static` (`geometry_msgs/msg/TransformStamped`) | Static transforms to transform the incoming pointcloud to the configured `child_frame` which the output odometry shall be in |
| `/vehicle/sensor/lidar` (`sensor_msgs/msg/Pointcloud2`) | Input pointcloud topic |
| [[OPTIONAL]] `/core/orchestration/lidar_status` (`diagnostic_msgs/msg/DiagnosticStatus`) | Only if input status check enabled (`diagnostic.check_input_status`). Corresponding status to pointcloud topic as is e.g. from [tam_odometry](tam_odometry.md) |
| [[OPTIONAL]] `/tf` (`geometry_msgs/msg/TransformStamped`) | Only required if using an external prediction model (executable contains `EXT`) or undistortion. Contains the transformation of the robot (`child_frame`) in the environment (`odom_frame`), used as initial guess for the registration or for twist interpolation. |
| [[OPTIONAL]] `/core/map/get_pointcloud_map` (`tum_map_msgs/srv/GetPointCloudMap`) | Use to request an initial prior pointcloud map within the software stack. Only applied if `node.input_map` does not point to a `.pcd`-file and `pipeline.update_map = false`. |
<!-- markdownlint-enable MD013 -->

<!-- markdownlint-disable MD013 -->
| Published Topics (Message Type) | Description |
| -------------------------------- | ----------- |
| `/core/state/lidar_odometry` (`nav_msgs/msg/Odometry`) | Registered odometry poses. |
| `/core/orchestration/OdometryNode_status` (`diagnostic_msgs/msg/DiagnosticStatus`) | Corresponding status of registered pose. |
| `/debug/core/state/OdometryNode` (`tsl_msgs/TSLValues`) | Internal debug signals with data on timing, iterations, diagnostics etc. |
| `/debug/core/state/OdometryNode/def` (`tsl_msgs/TSLDefinition`) | Definitions for internal debug signals (required for parsing). |
<!-- markdownlint-enable MD013 -->

### Usage

This section briefly explains how to build and run the `tam_odometry` node standalone.

1. Clone [FAR-LIO](https://github.com/TUMFTM/FAR-LIO) into the `src` folder of a ROS2 workspace
   and build it:

    ```bash
    colcon build
    source install/setup.bash
    ```

2. Start the `tam_odometry` package with a given executable/component (e.g. `tam_icp_cv`, see
   available [components](#available-ros2-components)) and pass a parameter file (see available
   [modes](#available-modes)):

    ```bash
    ros2 run tam_odometry tam_icp_cv --ros-args -r __ns:=/core/state --params-file <path/to/config.yml>
    ```
