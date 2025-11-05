# ============================================================
# ROS 2 ORB-SLAM3 (Jazzy) — Native ARM64 build for Qualcomm RB3
# ============================================================

FROM ros:jazzy

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/root/ros2_ws
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ------------------------------------------------------------
# System dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    ninja-build \
    python3-pip \
    python3-numpy \
    python3-opencv \
    pkg-config \
    wget \
    \
    # Core C++ / system libs
    libeigen3-dev \
    libssl-dev \
    libc++-dev \
    \
    # Graphics + GUI support
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libglew-dev \
    libepoxy-dev \
    libxi-dev \
    libxmu-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    \
    # Image / video IO
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libdc1394-dev \
    libraw1394-dev \
    libopenexr-dev \
    libv4l-dev \
    \
    # Boost (fixes DBoW2 missing serialization.hpp)
    libboost-all-dev \
    \
    # ROS interfaces
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf2-ros \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Python dependencies for colcon and build helpers
# ------------------------------------------------------------
RUN pip3 install --no-cache-dir --break-system-packages \
    colcon-common-extensions \
    empy \
    pybind11 

# ------------------------------------------------------------
# Pangolin (for ORB-SLAM3 viewer)
# ------------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    cd .. && rm -rf Pangolin

RUN echo "/usr/local/lib" >> /etc/ld.so.conf.d/pangolin.conf && ldconfig
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# ------------------------------------------------------------
# ROS 2 workspace and ORB-SLAM3 repo
# ------------------------------------------------------------
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}
COPY . src/ros2_orb_slam3/

# ---------------------------
# Fix CMakeLists and OpenCV compatibility issues
# ---------------------------
# Fix DBoW2 CMakeLists.txt path and typo
RUN sed -i -e 's|set (dbow2_ROOR_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2")|set(dbow2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")|' \
           -e 's|dbow2_ROOR_DIR|dbow2_ROOT_DIR|g' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt

# Add proper CMake configuration to DBoW2
RUN sed -i '1i\
cmake_minimum_required(VERSION 3.5)\n\
project(DBoW2)\n\
find_package(OpenCV REQUIRED)\n\
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt

# Fix g2o CMakeLists.txt if needed
RUN sed -i 's|${PROJECT_SOURCE_DIR}/${PROJECT_SOURCE_DIR}|${PROJECT_SOURCE_DIR}|g' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt || true

# Fix OpenCV 4 header paths in DBoW2
RUN find src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2 -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -exec sed -i \
    -e 's|opencv2/core/core.hpp|opencv2/core.hpp|g' \
    -e 's|opencv2/highgui/highgui.hpp|opencv2/highgui.hpp|g' \
    -e 's|opencv2/imgproc/imgproc.hpp|opencv2/imgproc.hpp|g' \
    -e 's|opencv2/features2d/features2d.hpp|opencv2/features2d.hpp|g' \
    -e 's|opencv2/calib3d/calib3d.hpp|opencv2/calib3d.hpp|g' \
    {} +

# Fix OpenCV 4 header paths in g2o  
RUN find src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -exec sed -i \
    -e 's|opencv2/core/core.hpp|opencv2/core.hpp|g' \
    -e 's|opencv2/highgui/highgui.hpp|opencv2/highgui.hpp|g' \
    -e 's|opencv2/imgproc/imgproc.hpp|opencv2/imgproc.hpp|g' \
    -e 's|opencv2/features2d/features2d.hpp|opencv2/features2d.hpp|g' \
    -e 's|opencv2/calib3d/calib3d.hpp|opencv2/calib3d.hpp|g' \
    {} +

# ---------------------------
# Build third-party dependencies from source
# ---------------------------
RUN cd src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2 && \
    rm -rf build && \
    mkdir -p build && cd build && \
    cmake -S .. -B . && make -j1

# Debug: Check the g2o CMakeLists.txt content
RUN echo "=== Checking g2o CMakeLists.txt for path issues ===" && \
    grep -n "PROJECT_SOURCE_DIR\|ADD_LIBRARY\|types_sba" \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt || true

# ---------------------------
# Patch g2o CMakeLists.txt and build
# ---------------------------
RUN sed -i '1i cmake_minimum_required(VERSION 3.5)\nproject(g2o)' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt && \
    # Add Eigen include
    sed -i '/project(g2o)/a include_directories(/usr/include/eigen3)' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt && \
    # Fix configure_file to use correct path
    sed -i 's|configure_file(.*config.h.in.*)|configure_file(config.h.in config.h)|' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt && \
    # Fix the g2o_SOURCE_DIR to not duplicate the path
    sed -i 's|set(g2o_SOURCE_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o")|set(g2o_SOURCE_DIR "${PROJECT_SOURCE_DIR}")|' \
    src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/CMakeLists.txt

# Build g2o
RUN cd src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o && \
    rm -rf build && mkdir -p build && cd build && \
    cmake -S .. -B . \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-I/usr/include/eigen3" && \
    make -j$(nproc)

# Fix OpenCV version mismatch (g2o expects 4.5d)
RUN OPENCV_LIB=$(find /usr/lib -name "libopencv_core.so.4*" | head -1) && \
    if [ -n "$OPENCV_LIB" ]; then \
        ln -sf $OPENCV_LIB /usr/lib/$(uname -m)-linux-gnu/libopencv_core.so.4.5d; \
    fi

# Fix OpenCV 4 header paths in main ORB-SLAM3 code (before colcon build)
RUN find src/ros2_orb_slam3 -path src/ros2_orb_slam3/orb_slam3/Thirdparty -prune -o -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.cc" \) -print -exec sed -i \
    -e 's|opencv2/core/core.hpp|opencv2/core.hpp|g' \
    -e 's|opencv2/highgui/highgui.hpp|opencv2/highgui.hpp|g' \
    -e 's|opencv2/imgproc/imgproc.hpp|opencv2/imgproc.hpp|g' \
    -e 's|opencv2/features2d/features2d.hpp|opencv2/features2d.hpp|g' \
    -e 's|opencv2/calib3d/calib3d.hpp|opencv2/calib3d.hpp|g' \
    -e 's|opencv2/video/tracking.hpp|opencv2/video.hpp|g' \
    {} +

RUN find /opt/ros/${ROS_DISTRO} -name "cv_bridge.h" -o -name "cv_bridge.hpp"

# ------------------------------------------------------------
# Install ROS dependencies and build
# ------------------------------------------------------------
#RUN apt-get update && \
#    . /opt/ros/${ROS_DISTRO}/setup.sh && \
#    rosdep update && \
#    rosdep install -r --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} && \
#    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Fix cv_bridge include paths - use .hpp not .h
# ------------------------------------------------------------
RUN echo "=== Fixing cv_bridge includes (using .hpp) ===" && \
    find src/ros2_orb_slam3 -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" \) \
    -exec sed -i \
        -e 's|#include <cv_bridge/cv_bridge\.h>|#include <cv_bridge/cv_bridge/cv_bridge.hpp>|g' \
        -e 's|#include <cv_bridge/cv_bridge\.hpp>|#include <cv_bridge/cv_bridge/cv_bridge.hpp>|g' \
        -e 's|#include "cv_bridge/cv_bridge\.h"|#include <cv_bridge/cv_bridge/cv_bridge.hpp>|g' \
        -e 's|#include "cv_bridge/cv_bridge\.hpp"|#include <cv_bridge/cv_bridge/cv_bridge.hpp>|g' \
        {} +

# Verify the fix
RUN echo "=== Verifying cv_bridge includes ===" && \
    grep -n "cv_bridge" src/ros2_orb_slam3/include/ros2_orb_slam3/common.hpp | head -5

# ------------------------------------------------------------
# Fix image_transport include paths - use .hpp not .h
# ------------------------------------------------------------
RUN echo "=== Fixing image_transport includes (using .hpp) ===" && \
    find src/ros2_orb_slam3 -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" \) \
    -exec sed -i \
        -e 's|#include <image_transport/image_transport\.h>|#include <image_transport/image_transport.hpp>|g' \
        -e 's|#include "image_transport/image_transport\.h"|#include <image_transport/image_transport.hpp>|g' \
        {} +

# Verify the fix
RUN echo "=== Verifying image_transport includes ===" && \
    grep -rn "image_transport/image_transport" src/ros2_orb_slam3/include/ || true

# Build with verbose output to see errors
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    MAKEFLAGS="-j2" colcon build --symlink-install \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_CXX_FLAGS="-I/opt/ros/${ROS_DISTRO}/include" \
                 -DCMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}"

# ------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${WORKSPACE}/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

LABEL description="ROS 2 ORB-SLAM3 (Jazzy) for Qualcomm RB3 — Native ARM64"
LABEL maintainer="Based on Mechazo11/ros2_orb_slam3"
