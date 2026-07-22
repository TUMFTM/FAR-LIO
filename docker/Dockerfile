# =============================================================================
# FAR-LIO container image
#
# Three stages:
#   * deps    – CUDA devel base (nvcc + version-matched CUDA libraries) + ROS 2 +
#               every system/ROS dependency needed to BUILD the nodes. Installed
#               explicitly so that every package pulled into the image
#               is visible in this file.
#   * build   – deps + the FAR-LIO sources; compiles and installs the nodes into
#               /dev_ws/install. Intermediate (not published on its own).
#   * runtime – slim CUDA *runtime* base + only the RUN-time ROS/PCL/CUDA libraries
#               the nodes actually link against, plus the compiled /dev_ws/install
#               (copied from the build stage) and /dev_ws/config (from the build
#               context). This is the published image. The heavy devel toolchain
#               (nvcc, cmake, colcon, *-dev headers, ~19 GB) stays in the build
#               stage and is discarded.
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
# CUDA architectures to compile device code for (only used when BUILD_CUDA=ON).
# A ';'-separated list produces multi-architecture e.g.
#   89  -> Ada    (RTX 40xx / L40)
#   120 -> Blackwell (RTX 50xx)
ARG CUDA_ARCHITECTURES=89

WORKDIR /dev_ws

# Copy sources, preserving the `ros_packages` <-> `lib_cpp` sibling layout that
# the CMakeLists rely on (add_subdirectory ../../lib_cpp/...).
COPY ros_packages/ src/far_lio/ros_packages/
COPY lib_cpp/      src/far_lio/lib_cpp/
COPY submodules/   src/far_lio/submodules/

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

# =============================================================================
# Stage 3: runtime  – slim image with only the runtime dependencies + install
# =============================================================================
FROM nvidia/cuda:${CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION} AS runtime

ARG ROS_DISTRO
ARG TZ=Etc/UTC

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    TZ=${TZ} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=/dev_ws/config/middleware/cyclonedds_udp.xml

# --- Locale + ROS 2 apt repository (same setup as the deps stage) -----------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

# Runtime-only dependencies (no compilers, no *-dev headers):
#   * ros-base            – rclcpp/rclcpp_components/tf2/message_filters + ros2cli
#                           (provides `ros2 component types`) and the sensor/nav/
#                           geometry/diagnostic message runtime libraries.
#   * rmw-cyclonedds-cpp  – the RMW selected via RMW_IMPLEMENTATION.
#   * geographic-msgs     – geographic_msgs runtime typesupport libraries.
#   * libpcl-io1.14       – PCL runtime the nodes link (libpcl_common / libpcl_io /
#                           libpcl_io_ply); pulls its own VTK/OpenNI/boost runtime.
# pcl_conversions is header-only (no runtime lib) and TBB ships inside install/.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-geographic-msgs \
      libpcl-io1.14 \
    && rm -rf /var/lib/apt/lists/*

# Copy the compiled workspace from the build stage and the runtime configuration
# from the build context.
COPY --from=build /dev_ws/install /dev_ws/install

WORKDIR /dev_ws

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
