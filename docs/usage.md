# Usage

FAR-LIO is run through Docker Compose. The provided `docker-compose.yml` defines two
services, each behind a [profile](https://docs.docker.com/compose/profiles/):

| Service | Profile | Purpose |
|---------|---------|---------|
| `far-lio-core` | `far-lio` | Runs the LiDAR odometry (CUDA GICP) and 3D-EKF state-estimation nodes. Requires a GPU. |
| `rosbag` | `bag` | Plays a ROS 2 bag to feed the stack (`ros2 bag play … --clock`). |

Both services use host networking and CycloneDDS, so they communicate with each other —
and anything else on the host — over `ROS_DOMAIN_ID`.

The image, RMW implementation (`rmw_cyclonedds_cpp`) and CPU pinning (`cpuset: "0-3"`) are
fixed in the compose file. `ROS_DOMAIN_ID` is taken from your shell environment if set, and
otherwise defaults to `0`.

## Prerequisites

Before running FAR-LIO on your own bag or sensors, configure it for your data in
`config/far-lio.yml`: set the LiDAR `input_pointcloud` topic, the IMU topic, and the
`cloud_frame` (the `child_frame` / `odom_frame` are fixed to the vehicle frame — leave them
as-is). Your bag must also provide the static transform `cloud_frame → base_link` on
`/tf_static`. See [Configuration](configuration.md) for details.

!!! note
    The `StateEstimation` → `LidarOdometry` interface runs over TF: `StateEstimation` publishes
    the fused vehicle pose on `/tf`, and `LidarOdometry` looks up its initial guess for
    registration from there. Your bag must therefore **not contain its own `/tf`** (dynamic
    transforms) — it would overwrite FAR-LIO's pose and corrupt the odometry. `/tf_static` with
    the sensor extrinsics is fine, and required.

## Run FAR-LIO on its own

For a live setup (real sensors publishing on the host):

```bash
docker compose --profile far-lio up
```

Stop with `Ctrl-C` (or `docker compose --profile far-lio down`).

## Run FAR-LIO with a bag

To directly run `far-lio` on your rosbag, use the
`run.sh` helper. It takes the bag path, mounts it into the `rosbag` container, and starts
both services:

```bash
./run.sh /path/to/rosbag
```

`/path/to/rosbag` may be a bag directory (containing `metadata.yaml`) or an `.mcap` file;
both mcap and sqlite3 storage are supported.

To use a specific `ROS_DOMAIN_ID`, export it first:

```bash
ROS_DOMAIN_ID=7 ./run.sh /path/to/rosbag
```

## Test Data

We provide a short example dataset of `KITTI Odometry (sequence 04)` in the respective ROS2 format for FAR-LIO to check system functionality.
It is available as an asset and can be downloaded from the respective release using:

```bash
cd /path/to/far-lio
curl -fL -O https://github.com/TUMFTM/FAR-LIO/releases/download/test-data-kitti-seq04/test-data-kitti-seq04.zip && unzip test-data-kitti-seq04.zip
```

The default configuration in `far-lio.yml` already points at the correct inputs for the test data. To run `FAR-LIO`, use:

```bash
./run.sh ./test-data-kitti-seq04
```

For visualization/inspection, run [PlotJuggler](https://github.com/facontidavide/PlotJuggler) and subscribe to the respective topics, or open RVIZ2 with the provided config:

```bash
rviz2 -d config/far-lio.rviz
```

For further details, see [Analysis](analysis.md).
