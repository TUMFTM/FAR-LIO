# Configuration

All parameters live in [`config/far-lio.yml`](https://github.com/TUMFTM/FAR-LIO/blob/main/config/far-lio.yml).
The compose setup bind-mounts the `config/` directory into the container at `/config` and both
nodes are started with `--params-file /config/far-lio.yml`, so you can edit the file on the host
and simply restart — no rebuild needed.

The file has one block per node, both under the `/core/state` namespace:

- `LidarOdometry` — the CUDA GICP odometry node.
- `StateEstimation` — the 3D-EKF backend.

## Running on a new bag

At minimum you must point FAR-LIO at the **topics** in your bag and set the **frames** that
match your sensor setup.

### Topics

| Node | Parameter | Type | Meaning |
|------|-----------|------|---------|
| `LidarOdometry` | `node.input_pointcloud` | `sensor_msgs/PointCloud2` | LiDAR point-cloud topic to register. **Required.** |
| `StateEstimation` | `inputs.imus.primary.message.topic` | `sensor_msgs/Imu` | IMU topic fused in the EKF. **Required.** |
| `LidarOdometry` | `node.output_odom` | `nav_msgs/Odometry` | Odometry output topic (default `lidar_odometry`; the EKF subscribes to it — usually leave as is). |

```yaml
/core/state:
  LidarOdometry:
    ros__parameters:
      node:
        input_pointcloud: "/sensor/lidar/points"   # <- your PointCloud2 topic
  StateEstimation:
    ros__parameters:
      inputs:
        imus:
          primary:
            message:
              topic: "/sensor/imu/data"            # <- your Imu topic
```

### Frames

The state estimation operates in the vehicle's center of gravity, so `child_frame` and
`odom_frame` are assumed fixed — **leave them at their defaults**. You only need to set
`cloud_frame` to match your sensor.

| Parameter | Meaning |
|-----------|---------|
| `node.cloud_frame` | **Set this** to the `frame_id` of your LiDAR `PointCloud2`. The node blocks until the static transform `cloud_frame → child_frame` is available on `/tf_static` and transforms each incoming cloud before registration. |
| `node.child_frame` | Vehicle body / center-of-gravity frame the estimator works in (`base_link`). **Do not change.** |
| `node.odom_frame` | Fixed world/odometry frame the pose is published in (`local_cartesian`). **Do not change.** |

```yaml
/core/state:
  LidarOdometry:
    ros__parameters:
      node:
        odom_frame: "local_cartesian"   # keep
        child_frame: "base_link"        # keep
        cloud_frame: "lidar_link"       # <- set to your PointCloud2 frame_id
        wait_tf: false
```

!!! important
    FAR-LIO needs the static transform from `cloud_frame` to `base_link` (on `/tf_static`) to
    place the LiDAR relative to the vehicle, so **your bag must contain it**. Find a cloud's
    frame with `ros2 topic echo --field header.frame_id <pointcloud_topic>`. If the bag has no
    suitable `/tf_static`, you can inject one — e.g. with
    [kappe](https://github.com/sensmore/kappe).

## Odometry vs. localization mode

`LidarOdometry` can either build a map online or localize against a prior map:

| Parameter | Effect |
|-----------|--------|
| `pipeline.update_map: true` | **Odometry mode** — builds and continuously updates a local map (start at the origin). |
| `pipeline.update_map: false` + `node.input_map` | **Localization mode** — loads a prior map from the given folder (containing `map.pcd` and `origin.yml`) and localizes against it. |

## Other useful parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Use `/clock` (required for bag playback; `run.sh` plays bags with `--clock`). Set `false` for live sensors. |
| `map.voxel_size` | `4.0` | Voxel edge length of the `cuVoxelMap` (m). |
| `map.max_distance` | `1000.0` | Voxels farther than this from the current pose are pruned (m). |
| `registration.max_time` | `200.0` | Per-scan registration time budget (ms). |
| `registration.convergence_criterion` | `5.0e-3` | GICP convergence threshold on the pose update. |
| `preprocessing.crop_range` | `[0.0]` | `[min, max]` range crop in m (disabled unless size 2). |
| `preprocessing.crop_footprint` | `[0.0]` | `[longitudinal, lateral]` vehicle-body crop in m (disabled unless size 2). |
