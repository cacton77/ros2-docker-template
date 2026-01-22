# MoveIt2 Python Docker

A Docker-based environment providing MoveIt2 with Python bindings (`moveit_py`) for ROS 2 Humble, built from source to enable functionality not available in standard apt packages. Designed to work both as a standalone development environment and as an overlay for Isaac ROS.

## Features

- **ROS 2 Humble** with full desktop installation
- **MoveIt2 with moveit_py** built from source (Python bindings unavailable in Humble apt packages)
- **Isaac ROS Compatible** - Works as an overlay for Isaac ROS development
- **GUI Support** for RViz and other visualization tools
- **Dual Usage**: Works standalone or as Isaac ROS overlay
- **Development Ready** with essential tools (neovim, build tools, etc.)

## Quick Start

### Standalone Usage

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd moveit2-py-docker
   ```

2. **Enable GUI applications (Linux only):**
   ```bash
   xhost +local:docker
   ```

3. **Build and run:**
   ```bash
   docker compose build
   
   # Basic container
   docker compose up moveit2
   ```

4. **Access the container:**
   ```bash
   docker exec -it moveit2 bash
   ```

5. **Test MoveIt2 Python bindings:**
   ```bash
   # Inside the container - test that moveit_py is available
   python3 -c "import moveit; print('MoveIt2 Python bindings available!')"
   
   # Run MoveIt2 demo
   ros2 launch moveit2_tutorials demo.launch.py
   ```

### Isaac ROS Overlay Usage

To use this as an overlay for Isaac ROS development:

1. **Update your Isaac ROS build script** to use this Dockerfile as a base:
   ```bash
   # In your Isaac ROS workspace
   cp path/to/Dockerfile.moveit2 src/isaac_ros_common/docker/
   ```

2. **Modify the Isaac ROS build command:**
   ```bash
   cd ${ISAAC_ROS_WS}/src/isaac_ros_common
   ./scripts/run_dev.sh --dockerfile Dockerfile.moveit2
   ```

3. **The container will include both Isaac ROS and MoveIt2** with all dependencies resolved.

## Repository Structure

```
moveit-py-docker/
├── README.md                 # This file
├── docker-compose.yaml       # Standalone Docker Compose setup
├── Dockerfile.moveit2        # Main Dockerfile with MoveIt2 + moveit_py
├── workspace/               # Shared workspace directory
└── config/                  # Configuration files (for dev profile)
```

## Docker Compose Services

### `moveit2` (Default)
- **Purpose**: Basic MoveIt2 development environment
- **Volumes**: `./workspace` mounted to `/workspaces/shared`
- **Ports**: Host networking for ROS communication
- **GUI**: X11 forwarding enabled

### `moveit2-dev` (Development Profile)
- **Purpose**: Enhanced development with additional configuration
- **Activation**: `docker-compose --profile dev up -d moveit2-dev`
- **Additional Volumes**: `./config` mounted to `/workspaces/config`
- **Features**: Auto-sources MoveIt2 workspace on startup

## Environment Details

### Installed Packages
- ROS 2 Humble (full desktop)
- MoveIt2 with moveit_py (built from source, main branch)
- Python development tools
- Build essentials (cmake, colcon, etc.)
- OpenGL/GLUT libraries for 3D visualization
- Neovim text editor
- Additional ROS packages: osqp-vendor, stomp, topic-tools, rmf-utils

**Why built from source?** The moveit_py Python bindings are not available in the standard ROS 2 Humble apt packages, requiring a source build to access this functionality.

### Workspace Layout
- **MoveIt2 Workspace**: `/workspaces/base_ws`
- **Shared Directory**: `/workspaces/shared` (mounted from host)
- **Auto-sourcing**: MoveIt2 workspace is automatically sourced in bash

## Usage Examples

### Running MoveIt2 Demos

```bash
# Start the demo with RViz
ros2 launch moveit2_tutorials demo.launch.py

# Run planning scene tutorial
ros2 launch moveit2_tutorials planning_scene_tutorial.launch.py

# Test Python bindings availability
python3 -c "import moveit; print('✓ MoveIt Python bindings work!')"
```

### Developing with moveit_py

```bash
# Example Python script using moveit_py
python3 << 'EOF'
import moveit
from moveit.planning import MoveItPy

# Initialize MoveItPy
moveit_py = MoveItPy(node_name="moveit_py_demo")
print("✓ MoveItPy initialized successfully!")

# Access planning components
robot_model = moveit_py.get_planning_component("panda_arm")
print(f"✓ Planning component loaded: {robot_model}")
EOF
```

### Using with Your Own Code

Mount your code directory when running:

```bash
# Custom volume mount
docker run -it --rm \
  --gpus all \
  -v /path/to/your/code:/workspaces/my_code \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --net host \
  moveit2:humble bash
```

### Creating ROS 2 Packages

```bash
# Navigate to your workspace
cd /workspaces/shared

# Create a new Python package with moveit_py
ros2 pkg create --build-type ament_python my_moveit_package \
  --dependencies rclpy moveit_msgs geometry_msgs

# Build your package
cd /workspaces/base_ws
colcon build --packages-select my_moveit_package
```

## Troubleshooting

### GUI Applications Not Working

1. **Enable X11 forwarding:**
   ```bash
   xhost +local:docker
   ```

2. **Check DISPLAY variable:**
   ```bash
   echo $DISPLAY  # Should output something like :0
   ```

3. **For other platforms**, consider using VNC or X11 forwarding alternatives.

### Build Issues

1. **Check available memory** - MoveIt2 compilation requires significant RAM (8GB+ recommended)

2. **Clean build if needed:**
   ```bash
   cd /workspaces/base_ws
   rm -rf build/ install/ log/
   colcon build
   ```

### Isaac ROS Integration Issues

1. **Ensure compatible base images** - The Dockerfile uses `osrf/ros:humble-desktop-full` by default
2. **Check for conflicting packages** - Some Isaac ROS packages may conflict with MoveIt2 dependencies
3. **Build order matters** - Build Isaac ROS packages first, then MoveIt2 overlay

## Development

### Building Custom Images

```bash
# Build with custom base image
docker build -f Dockerfile.moveit2 \
  --build-arg BASE_IMAGE=your-custom-base:latest \
  --build-arg ROS_DISTRO=humble \
  -t your-moveit2:custom .
```

### Modifying the Build

The Dockerfile includes several build optimizations:
- **Sequential executor** to avoid memory issues
- **Release build** for better performance
- **Testing disabled** for faster builds
- **Selective package overrides** to handle conflicts

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test your changes with both standalone and Isaac ROS configurations
4. Submit a pull request

## License

This project follows the same licensing as MoveIt2 and ROS 2. See individual package licenses for details.

## Support

- **MoveIt2 Documentation**: https://moveit.picknik.ai/
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Isaac ROS Documentation**: https://nvidia-isaac-ros.github.io/

## Changelog

### v1.0.0
- Initial release with ROS 2 Humble and MoveIt2
- **moveit_py Python bindings** built from source
- Docker Compose configuration for standalone usage
- Isaac ROS overlay compatibility
- GUI support for visualization tools
- GLUT/OpenGL support for 3D perception
