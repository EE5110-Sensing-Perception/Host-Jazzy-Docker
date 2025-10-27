# ROS2 Workspace

A comprehensive ROS2 workspace for robotics development, featuring IMU-based odometry, visual-inertial SLAM, and various utility packages. This workspace supports both ROS2 Humble and Jazzy distributions with Docker containerization for easy deployment and development.

## Quick Start

### Prerequisites

- Docker and Docker Compose
- ROS2 (Humble or Jazzy) - if running natively
- Ubuntu 22.04 (for Humble) or Ubuntu 24.04 (for Jazzy)

### Using Docker (Recommended)

1. **Build the Docker image:**
   ```bash
   # For ROS2 Jazzy
   ./scripts/docker_build.sh
   
   # For ROS2 Humble
   docker build -t humble_ws:latest -f dockerfile/humble_ws.dockerfile .
   ```

2. **Run the container:**
   ```bash
   # Standard container
   ./scripts/docker_run.sh
   
   # NVIDIA GPU support
   ./scripts/docker_run_nvidia.sh
   ```

3. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

### Native Installation (not recommended)

1. **Install ROS2:**
   ```bash
   # For Humble (Ubuntu 22.04)
   sudo apt install ros-humble-desktop
   
   # For Jazzy (Ubuntu 24.04)
   sudo apt install ros-jazzy-desktop
   ```

2. **Build the workspace:**
   ```bash
   cd /path/to/ros2_ws
   colcon build
   source install/setup.bash
   ```


## Docker Configuration

### Available Dockerfiles

| Dockerfile | Base Image | ROS2 Version | Description |
|------------|------------|--------------|-------------|
| `humble_ws.dockerfile` | Ubuntu 22.04 | Humble | Standard Humble development environment |
| `jazzy_ws.dockerfile` | Ubuntu 24.04 | Jazzy | Standard Jazzy development environment |
| `nvidia_humble.dockerfile` | NVIDIA CUDA 11.7 + Ubuntu 22.04 | Humble | GPU-accelerated Humble environment |
| `nvidia_jazzy.dockerfile` | NVIDIA CUDA 11.7 + Ubuntu 24.04 | Jazzy | GPU-accelerated Jazzy environment |

### Docker Features

- **X11 Forwarding**: Support for GUI applications (RViz, RQt)
- **Volume Mounting**: Host code mounted for live development
- **GPU Support**: NVIDIA GPU acceleration for computer vision workloads
- **Network Access**: Host network mode for ROS2 communication
- **Development Tools**: Pre-installed debugging and development utilities

### Docker Scripts

- `scripts/docker_build.sh`: Build Docker images
- `scripts/docker_run.sh`: Run standard containers
- `scripts/docker_run_nvidia.sh`: Run GPU-accelerated containers

## Development

### Building Packages

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select your_package

# Build with specific arguments
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Running Tests

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select odom_imu

# View test results
colcon test-result --verbose
```

### Code Quality

```bash
# Lint Python code
ament_flake8 src/

# Lint C++ code
ament_cpplint src/

# Format code
ament_uncrustify src/
```

## Workspace Structure

```
ros2_ws/
├── src/                          # Source packages
├── build/                        # Build artifacts
├── install/                      # Installed packages
├── log/                          # Build and test logs
├── dockerfile/                   # Docker configurations
│   ├── humble_ws.dockerfile
│   ├── jazzy_ws.dockerfile
│   ├── nvidia_humble.dockerfile
│   ├── nvidia_jazzy.dockerfile
│   └── entrypoint.sh
└── scripts/                      # Utility scripts
    ├── docker_build.sh
    ├── docker_run.sh
    └── docker_run_nvidia.sh
```

## Common Use Cases

1. **Visualize in RViz:**
   ```bash
   rviz2
   ```

## Configuration

### Environment Variables

- `ROS_DISTRO`: ROS2 distribution (humble/jazzy)
- `DISPLAY`: X11 display for GUI applications
- `NVIDIA_VISIBLE_DEVICES`: GPU visibility in containers

### Package Dependencies

Key dependencies are automatically installed in Docker containers:
- ROS2 core packages
- OpenCV
- Eigen3
- GStreamer
- Navigation2
- SLAM Toolbox
- RViz2 and RQt

## Troubleshooting

### Common Issues

1. **Docker X11 Issues:**
   ```bash
   xhost +local:root
   ```

2. **Permission Issues:**
   ```bash
   sudo chmod +x scripts/*.sh
   ```

3. **Build Failures:**
   ```bash
   colcon build --cmake-clean-cache
   ```

4. **Package Not Found:**
   ```bash
   source install/setup.bash
   ```

### Debugging

1. **Check ROS2 environment:**
   ```bash
   printenv | grep ROS
   ```

2. **List available packages:**
   ```bash
   ros2 pkg list
   ```

3. **Check node status:**
   ```bash
   ros2 node list
   ros2 topic list
   ```

## Documentation

- [ROS2 Documentation](https://docs.ros.org/en/humble/) - Official ROS2 documentation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

- base package uses MIT licenses
- 3rd party packages - TBC

## Maintainers

- **Brian Deegan** - brian.deegan@universityofgalway.ie

## 🔗 Related Projects

- to be added

**Note**: This workspace is designed for research and development purposes. Please ensure proper testing and validation before deploying in production environments.
