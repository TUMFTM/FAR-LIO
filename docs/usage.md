# Usage

FAR-LIO is run through Docker Compose. The provided `docker-compose.yml` defines two
services, each behind a [profile](https://docs.docker.com/compose/profiles/):

| Service | Profile | Purpose |
|---------|---------|---------|
| `far-lio-core` | `far-lio` | Runs the LiDAR odometry (CUDA GICP) and 3D-EKF state-estimation nodes. Requires a GPU. |
| `rosbag` | `bag` | Plays a ROS 2 bag to feed the stack (`ros2 bag play … --clock`). |
| `robot-state-publisher` | `rsp` | Publishes the sensor extrinsics from a URDF onto `/tf_static`. Optional — use when your bag does not already provide `/tf_static` (see [Configuration](configuration.md)). |

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

## Publishing extrinsics from a URDF

If your bag does not already carry the sensor extrinsics on `/tf_static`, the optional
`robot-state-publisher` service can publish them from a URDF. Drop the URDF into
`config/urdf/` (an example, `config/urdf/KITTI.xml`, is included) and either add the profile
to a manual run or set `WITH_RSP=1` when using the helper script:

```bash
# with the helper script (KITTI.xml by default)
./run.sh --rsp /path/to/rosbag

# pick a different URDF from config/urdf/
URDF_NAME=my_robot.xml ./run.sh --rsp /path/to/rosbag

# or standalone
URDF_NAME=KITTI.xml docker compose --profile rsp up
```

The URDF must define the sensor frames as fixed joints relative to `base_link` (see the
bundled `KITTI.xml`); `robot_state_publisher` then latches those transforms on `/tf_static`.
Only the static extrinsics are published, so this does **not** conflict with the note above
about dynamic `/tf`.
