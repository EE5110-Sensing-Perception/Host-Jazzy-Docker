# Dockerfile for ROS2 ORB-SLAM3 (Jazzy branch)
# Based on Ubuntu 24.04 with ROS 2 Jazzy

FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/root/ros2_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    libeigen3-dev \
    libssl-dev \
    libglew-dev \
    libepoxy-dev \
    libpython3-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libxi-dev \
    libxmu-dev \
    wget \
    python3-pip \
    ninja-build \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libtiff-dev \
    libdc1394-dev \
    libraw1394-dev \
    libopenexr-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Pangolin
WORKDIR /tmp
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TOOLS=OFF \
        -DBUILD_EXAMPLES=OFF && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    cd .. && \
    rm -rf Pangolin

# Configure library path for Pangolin
RUN echo "/usr/local/lib" >> /etc/ld.so.conf.d/pangolin.conf && \
    ldconfig

# Set up LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Create workspace
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# Clone the repository
RUN cd src && \
    git clone -b jazzy https://github.com/Mechazo11/ros2_orb_slam3.git

# Create symbolic link for OpenCV compatibility issue
# This links OpenCV 4.6 to 4.5d which g2o expects
RUN OPENCV_LIB=$(find /usr/lib -name "libopencv_core.so.4*" | head -1) && \
    if [ -n "$OPENCV_LIB" ]; then \
        ln -sf $OPENCV_LIB /usr/lib/$(uname -m)-linux-gnu/libopencv_core.so.4.5d; \
    fi

# Install ROS dependencies
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Set up entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${WORKSPACE}/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# Metadata
LABEL description="ROS2 ORB-SLAM3 V1.0 for ROS 2 Jazzy"
LABEL maintainer="Based on Mechazo11/ros2_orb_slam3"
