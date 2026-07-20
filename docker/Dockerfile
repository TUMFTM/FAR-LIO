# syntax=docker/dockerfile:1
# =============================================================================
# FAR-LIO container image
#
# Three stages:
#   * deps     – CUDA runtime base + ROS 2 + the exact system/ROS libraries the
#                node links at runtime. Shared base for both other stages, so
#                the dependency set is defined once.
#   * builder  – deps + build toolchain and dev headers; compiles only
#                `tam_odometry` and its recursive in-workspace dependencies.
#   * runtime  – deps + the compiled `install/` space and runtime config.
#
# Dependencies are installed explicitly by hand (no rosdep) so that every
# package pulled into the image is visible in this file.
#
# CUDA is OPTIONAL. The default build is CPU-only and produces the executables
# `tam_icp_ext`, `tam_icp_cv` and `tam_gicp_ext`. Pass `--build-arg BUILD_CUDA=ON`
# (optionally with `CUDA_ARCHITECTURES=...`) to also build the CUDA nodes
# `tam_cuda_icp_ext` / `tam_cuda_gicp_ext`; the CUDA toolkit is then installed
# into the builder stage only.
# =============================================================================

ARG UBUNTU_VERSION=24.04
ARG CUDA_VERSION=12.8.1
ARG ROS_DISTRO=jazzy

# =============================================================================
# Stage 1: deps  – shared base with runtime dependencies only
# =============================================================================
FROM nvidia/cuda:${CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION} AS deps

ARG ROS_DISTRO
ARG TZ=Etc/UTC

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    TZ=${TZ} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# --- Locale + ROS 2 apt repository ------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

# --- Runtime dependencies (shared by builder and runtime) --------------------
# ros-base           : rclcpp, tf2, common_interfaces (sensor/geometry/nav/...)
# rmw-cyclonedds-cpp : the RMW selected below
# geographic-msgs    : pulled in transitively via tum_map_msgs
# libpcl-common/io   : the only PCL shared libs the component links at runtime
#                      (they drag in the VTK / OpenNI / Boost chain). TBB is
#                      vendored via FetchContent and ships inside install/.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-geographic-msgs \
      libpcl-common1.14 \
      libpcl-io1.14 \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Stage 2: builder  – deps + build toolchain, compiles the workspace
# =============================================================================
FROM deps AS builder

ARG ROS_DISTRO
# CUDA build toggle (OFF => CPU-only, no CUDA sources compiled).
ARG BUILD_CUDA=OFF
# Only used when BUILD_CUDA=ON (e.g. 80 for Ampere, 89 for Ada).
ARG CUDA_ARCHITECTURES=80
# apt suffix of the CUDA toolkit to match the base image (12.8.1 -> 12-8).
ARG CUDA_TOOLKIT_VERSION=12-8

# --- Build toolchain + dev headers -------------------------------------------
# build-essential/cmake : compilers and build system
# git                   : required by odometry_cpp's FetchContent deps
#                         (Sophus, oneTBB, robin-map, GoogleTest)
# colcon                : workspace build tool
# libeigen3-dev         : find_package(Eigen3) in odometry_cpp
# libpcl-dev            : find_package(PCL) headers + cmake config
# pcl-conversions       : find_package(pcl_conversions) in tam_odometry
# All other ROS build dependencies (ament_cmake, rclcpp, tf2, the *_msgs,
# rosidl generators, ...) are provided by ros-base from the deps stage.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      libeigen3-dev \
      libpcl-dev \
      ros-${ROS_DISTRO}-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# --- CUDA toolkit (only for CUDA builds; stays out of the runtime image) -----
RUN if [ "${BUILD_CUDA}" = "ON" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        cuda-toolkit-${CUDA_TOOLKIT_VERSION} && \
      rm -rf /var/lib/apt/lists/*; \
    fi

WORKDIR /dev_ws

# Copy sources, preserving the `ros_packages` <-> `lib_cpp` sibling layout that
# tam_odometry/CMakeLists.txt relies on (add_subdirectory ../../lib_cpp/...).
COPY ros_packages/ src/far_lio/ros_packages/
COPY lib_cpp/      src/far_lio/lib_cpp/
COPY submodules/   src/far_lio/submodules/

# --- Build only the node and its recursive in-workspace dependencies ---------
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
      --packages-up-to tam_odometry \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_CUDA=${BUILD_CUDA} \
        -DCUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
        -DBUILD_ODOMETRY_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
      --event-handlers console_direct+

# =============================================================================
# Stage 3: runtime  – deps + compiled workspace
# =============================================================================
FROM deps AS runtime

ARG ROS_DISTRO

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=/dev_ws/config/middleware/cyclonedds_udp.xml

WORKDIR /dev_ws

# Ship the compiled workspace and the runtime config.
COPY --from=builder /dev_ws/install /dev_ws/install
COPY config/ /dev_ws/config/

# Source ROS + the FAR-LIO overlay for every interactive/`docker run` shell.
RUN echo '. /opt/ros/'"${ROS_DISTRO}"'/setup.bash' >> /etc/bash.bashrc && \
    echo '. /dev_ws/install/setup.bash'            >> /etc/bash.bashrc && \
    ldconfig

CMD ["bash"]
