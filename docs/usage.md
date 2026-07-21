# Usage

FAR-LIO is run through Docker Compose. The provided `docker-compose.yml` defines two
services, each behind a [profile](https://docs.docker.com/compose/profiles/) so they can
be started independently:

| Service | Profile | Purpose |
|---------|---------|---------|
| `far-lio-core` | `far-lio` | Runs the LiDAR odometry (CUDA GICP) and 3D-EKF state-estimation nodes. Requires a GPU. |
| `rosbag` | `bag` | Plays a ROS 2 bag to feed the stack (`ros2 bag play … --clock`). |

Both services use host networking and CycloneDDS, so they communicate with each other —
and anything else on the host — over the configured `ROS_DOMAIN_ID`.

## 1. Configure the environment

Copy the example env file and adjust it:

```bash
cp .env.example .env
```

| Variable | Default | Description |
|----------|---------|-------------|
| `FARLIO_IMAGE` | `ghcr.io/tumftm/far-lio:latest` | Image to run. |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain (passed through to both services). |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Middleware (the image defaults to CycloneDDS). |
| `CPU_CORES` | `0-3` | Cores `far-lio-core` is pinned to (`cpuset`) for reproducible benchmarking. |
| `BAG_PATH` | `./bags` | Host directory mounted read-only into the `rosbag` service at `/bags`. |
| `BAG_NAME` | `recording` | Bag to play (a bag directory with `metadata.yaml`, or an `.mcap` file) under `BAG_PATH`. |

## 2. Launch

```bash
# FAR-LIO odometry + state estimation (needs a GPU)
docker compose --profile far-lio up

# Replay a bag (in a second terminal, or add the profile to the same command)
docker compose --profile bag up

# Both together
docker compose --profile far-lio --profile bag up
```

Stop with `Ctrl-C`, or `docker compose --profile far-lio --profile bag down`.

## How it works

`far-lio-core` starts both node executables and waits on them:

```bash
/dev_ws/install/tam_odometry/lib/tam_odometry/tam_cuda_gicp_ext \
  --ros-args -r __ns:=/core/state -r __node:=LidarOdometry --params-file /config/far-lio.yml &
/dev_ws/install/tam_state_estimation_node/lib/tam_state_estimation_node/tam_state_estimation_node_ekf_3d_nh \
  --ros-args -r __ns:=/core/state -r __node:=StateEstimation --params-file /config/far-lio.yml &
wait
```

- The node **namespace/name** (`/core/state` → `LidarOdometry` / `StateEstimation`) match the
  keys in `config/far-lio.yml`.
- Parameters are loaded from `config/far-lio.yml`, mounted into the container at `/config`.
- The nodes run with `use_sim_time: true`, which is why the bag is played with `--clock`.
- The CUDA GICP node needs a real GPU (the Compose file reserves one via the NVIDIA runtime).

## Playing your own data

Point `BAG_PATH`/`BAG_NAME` at your recording. Both mcap and sqlite3 bags are supported.
If your sensor topics differ from the defaults, add remaps to the `far-lio-core` command,
e.g. `-r /vehicle/sensor/imu:=<your_imu_topic>`.

## Running a single node manually

You can also run a node directly (the entrypoint sources the environment):

```bash
docker run --rm -it --gpus all --network host \
  -v "$PWD/config:/config" ghcr.io/tumftm/far-lio:latest \
  /dev_ws/install/tam_odometry/lib/tam_odometry/tam_cuda_gicp_ext \
  --ros-args -r __ns:=/core/state -r __node:=LidarOdometry --params-file /config/far-lio.yml
```
