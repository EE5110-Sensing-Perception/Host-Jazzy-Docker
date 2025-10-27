# Use NVIDIA-accelerated ROS 2 Jazzy base image
FROM nvidia/cuda:12.5.1-runtime-ubuntu24.04

ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    nano \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-rviz2 \
    python3-pip \
    python3-colcon-common-extensions \
    libusb-dev \
    libspnav-dev \
    libbluetooth-dev \
    libcwiid-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install transforms3d

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy and setup entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
