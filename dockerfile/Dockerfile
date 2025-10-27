# Start from the official ROS Humble ROS Core image
FROM osrf/ros:humble-desktop-full

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    wget \
    nano \
    ros-humble-cv-bridge \
    python3-cv-bridge \
    ros-humble-tf-transformations \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \ 
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # gstreamer dependencies
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    # joystick
    ros-humble-joy \
    ros-humble-joy-linux \
    ros-humble-teleop-twist-joy \
    # lidar
    ros-humble-rplidar-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages if needed
RUN pip3 install --no-cache-dir \
    numpy \
    megapi \
    transforms3d \
    pyserial
    # Add other Python packages here

# Source ROS environment in the bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Add any custom files or configurations
# COPY ./my_package /ros2_ws/src/my_package

# Build your workspace (if you added packages)
# RUN . /opt/ros/humble/setup.sh && colcon build

# Default command to run when starting the container
CMD ["bash"]


