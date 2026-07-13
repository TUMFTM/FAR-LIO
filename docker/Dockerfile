# Build argument for Ubuntu version 
# Default: 24.04 for jazzy/rolling
ARG UBUNTU_VERSION=24.04
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu${UBUNTU_VERSION}

# Build argument for ROS distribution (redeclared after FROM)
ARG ROS_DISTRO=jazzy
ARG TZ=Europe/Berlin

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TZ=${TZ}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}]: {message}"

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
  locales \
  curl \
  gnupg2 \
  lsb-release \
  ca-certificates \
  gpg \
  wget && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8 && \
  apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Add ROS 2 GPG key and repository
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
  dpkg -i /tmp/ros2-apt-source.deb

# Install dependencies
RUN apt-get update && \
    apt-get install -y -q --no-install-recommends \
    cmake \
    cmake-data \
    ros-$ROS_DISTRO-ros-base \
    ros-dev-tools \
    build-essential \
    ca-certificates \
    curl \
    gnupg2 \
    locales \
    lsb-release \
    git \
    vim \
    wget \
    unzip \
    gdb \
    nsight-systems-2025.5.2 \
    lttng-tools liblttng-ust-dev liblttng-ctl-dev \
    libboost-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libpcl-dev \
    iputils* \
    tmux \
    tmuxp \
    nano \
    libfmt-dev \
    libxext6 \
    libx11-6 \
    libglfw3-dev \
    libsfml-dev \
    libeigen3-dev \
    python3-venv \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    python3-babeltrace \
    python3-lttng \
    python3-dev \
    python3-numpy \
    python3-colcon-common-extensions \
    ipython3 \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-mimick-vendor \
    ros-$ROS_DISTRO-can-msgs \
    ros-$ROS_DISTRO-autoware-auto-msgs \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-ament-cmake-mypy \
    ros-$ROS_DISTRO-ros2-socketcan \
    ros-$ROS_DISTRO-mimick-vendor \
    ros-$ROS_DISTRO-test-msgs \
    ros-$ROS_DISTRO-ament-cmake-google-benchmark \
    ros-$ROS_DISTRO-performance-test-fixture \
    ros-$ROS_DISTRO-rclcpp-components \
    ros-$ROS_DISTRO-gps-tools \
    ros-$ROS_DISTRO-gps-msgs \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-ament-index-cpp \
    ros-$ROS_DISTRO-autoware-auto-msgs \
    ros-$ROS_DISTRO-joy* \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-ament-cmake-clang-format \
    libcanberra-gtk-module \
    libsdl2-ttf-dev \
    gfortran \
    liblapack-dev \
    pkg-config \
    swig \
    coinor-libipopt-dev \
    pyqt5-dev-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /dev_ws

# Copy config files
COPY config/ /dev_ws/config/

# Copy submodules
COPY submodules/ /dev_ws/src/

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/dev_ws/config/cyclonedds_$ROS_DISTRO.xml

# Add CMake to PATH
ENV PATH="/usr/local/bin:${PATH}"

# Make shared libraries visible to the system
RUN ldconfig

CMD ["bash"]
