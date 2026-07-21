# =============================================================================
# FAR-LIO container image
#
# Two stages:
#   * deps   – CUDA devel base (nvcc + version-matched CUDA libraries) + ROS 2 +
#              every system/ROS dependency needed to build and run the nodes.
#              Installed explicitly (no rosdep) so that every package pulled into
#              the image is visible in this file.
#   * build  – deps + the FAR-LIO sources; compiles and installs the nodes.
#

ARG UBUNTU_VERSION=24.04
ARG CUDA_VERSION=12.8.1
ARG ROS_DISTRO=jazzy

# =============================================================================
# Stage 1: deps
# =============================================================================
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} AS deps

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

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-geographic-msgs \
      ros-${ROS_DISTRO}-pcl-conversions \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      libeigen3-dev \
      libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Stage 2: build  – compile and install the FAR-LIO workspace
# =============================================================================
FROM deps AS build

ARG ROS_DISTRO
ARG BUILD_CUDA=ON
# Only used when BUILD_CUDA=ON (e.g. 80 for Ampere, 89 for Ada).
ARG CUDA_ARCHITECTURES=80

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=/dev_ws/config/middleware/cyclonedds_udp.xml

WORKDIR /dev_ws

# Copy sources, preserving the `ros_packages` <-> `lib_cpp` sibling layout that
# the CMakeLists rely on (add_subdirectory ../../lib_cpp/...).
COPY ros_packages/ src/far_lio/ros_packages/
COPY lib_cpp/      src/far_lio/lib_cpp/
COPY submodules/   src/far_lio/submodules/
COPY config/       /dev_ws/config/

# Build the nodes and their recursive in-workspace dependencies, then drop the
# intermediate build/ and log/ trees (only install/ is needed to run).
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
      --packages-up-to tam_odometry tam_state_estimation_node \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_CUDA=${BUILD_CUDA} \
        -DCUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
        -DBUILD_ODOMETRY_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
      --event-handlers console_direct+ && \
    rm -rf build log

# Source ROS + the FAR-LIO overlay for interactive shells (e.g. `docker exec`).
RUN echo '. /opt/ros/'"${ROS_DISTRO}"'/setup.bash' >> /etc/bash.bashrc && \
    echo '. /dev_ws/install/setup.bash'            >> /etc/bash.bashrc && \
    ldconfig

# Setup entrypoint
RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
      'if [ -f /dev_ws/install/setup.bash ]; then source /dev_ws/install/setup.bash; fi' \
      'exec "$@"' \
      > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
