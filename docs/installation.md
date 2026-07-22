# Installation

FAR-LIO ships as a single CUDA-enabled Docker image, so the only hard requirement on
the host is Docker. Running the CUDA nodes additionally needs an NVIDIA GPU with the
NVIDIA Container Toolkit.

## Prerequisites

- **Docker** (with Buildx / Compose v2).
- **NVIDIA GPU + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)** — required to *run* the CUDA odometry node.
- **git** — only needed if you build the image yourself (for the submodules).

## Option A — pull the pre-built image (recommended)

The image is published to the GitHub Container Registry on every push to `main`:

```bash
docker pull ghcr.io/tumftm/far-lio:latest
```

Release-tagged images (`ghcr.io/tumftm/far-lio:<tag>`) are published for each GitHub release.

## Option B — build it yourself

The build requires the submodules, so clone recursively:

```bash
git clone --recursive https://github.com/TUMFTM/FAR-LIO.git
cd FAR-LIO
# (if you cloned without --recursive)
git submodule update --init --recursive

docker build -f docker/Dockerfile -t ghcr.io/tumftm/far-lio:latest .
```

### Build arguments

| Arg | Default | Description |
|-----|---------|-------------|
| `BUILD_CUDA` | `ON` | Build the CUDA nodes (`tam_cuda_icp_ext`, `tam_cuda_gicp_ext`). |
| `CUDA_ARCHITECTURES` | `80` | Target GPU architecture(s). Set to match your GPU — e.g. `86` (RTX 30xx), `89` (RTX 40xx / Ada), `75` (RTX 20xx). A `;`-separated list is allowed. |
| `ROS_DISTRO` | `jazzy` | ROS 2 distribution. |
| `CUDA_VERSION` / `UBUNTU_VERSION` | `12.8.1` / `24.04` | Base image versions. |

```bash
# example: build for an Ada (RTX 40xx) GPU
docker build -f docker/Dockerfile \
  --build-arg CUDA_ARCHITECTURES=89 \
  -t ghcr.io/tumftm/far-lio:latest .
```

!!! note
    The published image is slim (~7 GB): it uses a CUDA *runtime* base and ships only the
    compiled workspace and the runtime libraries. The build itself is heavier — the `devel`
    toolchain stage is ~19 GB — so make sure you have enough free disk space while building.

## Option C — build from source (without Docker)

If you prefer a native ROS 2 workspace, you can build FAR-LIO directly on the host. The
[`docker/Dockerfile`](https://github.com/TUMFTM/FAR-LIO/blob/main/docker/Dockerfile) is the
always-up-to-date list of every dependency — install the same packages it does.

1. **ROS 2 Jazzy** on Ubuntu 24.04 (`ros-jazzy-ros-base`), plus the **CUDA toolkit** (12.8) if you
   want the CUDA nodes.
2. **System / ROS dependencies** (exactly as installed in the Dockerfile):
   `build-essential`, `cmake`, `git`, `python3-colcon-common-extensions`, `libeigen3-dev`,
   `libpcl-dev`, `ros-jazzy-pcl-conversions`, `ros-jazzy-geographic-msgs`,
   `ros-jazzy-rmw-cyclonedds-cpp`.

    !!! note
        Sophus, oneTBB, robin-map, GoogleTest and optionally cuCollections are fetched automatically at configure time via
        CMake `FetchContent`, so a network connection is required for the first build.

3. **Clone with submodules into a workspace and build** the two packages and their in-workspace
   dependencies:

    ```bash
    cd src
    git clone --recursive https://github.com/TUMFTM/FAR-LIO.git
    cd ros_ws

    source /opt/ros/jazzy/setup.bash
    colcon build --packages-up-to tam_odometry tam_state_estimation_node \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
                   -DBUILD_CUDA=ON -DCUDA_ARCHITECTURES=89 \
                   -DBUILD_ODOMETRY_EXAMPLES=OFF -DBUILD_TESTING=OFF
    source install/setup.bash
    ```

    Set `-DBUILD_CUDA=OFF` for the CPU-only nodes, and `CUDA_ARCHITECTURES` to match your GPU.
    See the [docker-compose.yml](https://github.com/TUMFTM/FAR-LIO/blob/main/docker-compose.yml) for running the resulting executables or the individual documentation of the node-packages [tam_odometry](./ros_packages/tam_odometry.md) and [tam_state_estimation_node](./ros_packages/tam_state_estimation_node.md).

## Docker Architecture

The image is built in three stages (see `docker/Dockerfile`):

- **`deps`** — CUDA *devel* base + ROS 2 `ros-base` + all build dependencies, installed
  explicitly so every package is visible in the Dockerfile.
- **`build`** — compiles only `tam_odometry` and `tam_state_estimation_node` and their
  in-workspace dependencies and installs them to `/dev_ws/install`.
- **`runtime`** — the published image: a slim CUDA *runtime* base with only the runtime
  libraries the nodes link against, plus the compiled `install/` copied from the build stage.

The entrypoint sources ROS 2 and the FAR-LIO overlay automatically, so any `docker run`
or Compose `command` works without sourcing anything first.
