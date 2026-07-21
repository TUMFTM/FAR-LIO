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
    The image is based on `nvidia/cuda:*-devel` and includes ROS 2, PCL and the CUDA
    toolkit, so it is large (~19 GB). Make sure you have enough disk space.

## What's inside

The image is built in two stages (see `docker/Dockerfile`):

- **`deps`** — CUDA devel base + ROS 2 `ros-base` + all system/ROS dependencies, installed
  explicitly (no `rosdep`) so every package is visible in the Dockerfile.
- **`build`** — compiles only `tam_odometry` and `tam_state_estimation_node` and their
  in-workspace dependencies (`colcon build --packages-up-to …`) and installs them to
  `/dev_ws/install`.

The entrypoint sources ROS 2 and the FAR-LIO overlay automatically, so any `docker run`
or Compose `command` works without sourcing anything first.
