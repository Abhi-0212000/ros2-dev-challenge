# ROS2 Wave Package

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Standard Installation](#standard-installation)
  - [Docker Installation](#docker-installation)
- [Usage](#usage)
  - [Running the Nodes](#running-the-nodes)
  - [Parameter Configuration](#parameter-configuration)
  - [Visualization Control](#visualization-control)
  - [Image Processing Service](#image-processing-service)
- [Development](#development)
- [Docker Reference](#docker-commands-reference)
- [License](#license)
- [Support](#support)

## Overview
A ROS2 package implementing a sine wave publisher/subscriber system with image processing capabilities, developed for the Next Generation Robotics Team ROS2 Developer Challenge. The system features configurable wave parameters, real-time visualization, and grayscale image conversion services.

## Features
- Sine wave generator with dynamically configurable parameters using generate_parameter_library
- Real-time visualization of sine wave data using matplotlib with adjustable buffer size
- Image processing service for grayscale conversion with OpenCV
- Side-by-side visualization of original and processed images
- Comprehensive test suite with unit and integration tests
- Docker support with X11 forwarding for visualization
- CI/CD pipeline with GitHub Actions for automated testing and linting and proper branch protection rules

## Installation

### Prerequisites

1. **Operating System Requirements**:
   - Ubuntu 22.04 (Jammy)
   - For Windows users: 
     1. First [enable WSL2](https://documentation.ubuntu.com/wsl/en/latest/howto/install-ubuntu-wsl2/)
     2. Then install [Ubuntu 22.04 from Microsoft Store](https://apps.microsoft.com/detail/9PN20MSR04DW)

2. **ROS 2 Installation**:
   - Follow the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
   - Make sure to follow all steps, including setting up your environment

3. **Required Tools**:
   ```bash
   # Install essential tools
   sudo apt-get install python3-rosdep python3-colcon-common-extensions python3-pip
   
   # Initialize and update rosdep
   sudo rosdep init
   rosdep update
   ```

### Standard Installation

1. **Set up ROS2 workspace**:
   ```bash
   # Create workspace directory
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   
   # Clone required repositories
   git clone https://github.com/Abhi-0212000/ros2-dev-challenge.git
   git clone https://github.com/picknikrobotics/generate_parameter_library.git
   
   # Install development dependencies. Which are not managed by rosdep. For code quality checking.
   cd ros2-dev-challenge
   pip3 install -r requirements-dev.txt
   ```

2. **Install dependencies and build**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

3. **Source the workspace**:
   ```bash
   # Source ROS2 and workspace (required for each new terminal)
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

4. **Optional: Persistent sourcing**:
   ```bash
   # Add to .bashrc for automatic sourcing in new terminals
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

5. **Verify installation**:
   ```bash
   # Run tests to ensure everything is working
   colcon test
   colcon test-result --verbose
   ```

### Docker Installation

#### Windows with GUI (Tested and Recommended)

1. **Prerequisites**:
   - Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
   - Follow this detailed [X11 forwarding and server setup guide](https://youtu.be/FlHVuA_98SA?t=151)

2. **Build and run**:
   ```powershell
   # Create directory and get files
   mkdir ros2_wave_docker
   cd ros2_wave_docker
   curl -O https://raw.githubusercontent.com/Abhi-0212000/ros2-dev-challenge/main/Dockerfile
   curl -O https://raw.githubusercontent.com/Abhi-0212000/ros2-dev-challenge/main/ros_entrypoint.sh
   
   # Build image
   docker build -t ros2_wave_pkg .
   
   # Verify image creation
   docker images
   
   # Run with X11 forwarding
   docker run -it --env="DISPLAY=host.docker.internal:0.0" --env="LIBGL_ALWAYS_INDIRECT=1" ros2_wave_pkg
   
   # Basic run without visualization
   docker run -it ros2_wave_pkg
   ```

#### Ubuntu with GUI

1. **Install Docker**:
   - Follow [Docker Desktop for Ubuntu installation guide](https://docs.docker.com/desktop/setup/install/linux/ubuntu/)
   - For server installations without GUI:
     - Install Docker Engine instead of Desktop
     - Enable X11 forwarding via SSH (ssh -X or ssh -Y)
     - Connect from a GUI-enabled client

2. **Build and run**:
   ```bash
   # Create directory and get files
   mkdir ros2_wave_docker && cd ros2_wave_docker
   wget https://raw.githubusercontent.com/Abhi-0212000/ros2-dev-challenge/main/Dockerfile
   wget https://raw.githubusercontent.com/Abhi-0212000/ros2-dev-challenge/main/ros_entrypoint.sh
   chmod +x ros_entrypoint.sh
   
   # Build image
   docker build -t ros2_wave_pkg .
   
   # Run with X11 forwarding
   docker run -it \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       --network=host \
       ros2_wave_pkg
   ```

If you encounter X11 permission issues:
```bash
xhost +local:docker
```

For more robust X11 forwarding:
```bash
docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    --network=host \
    --privileged \
    ros2_wave_pkg
```

## Usage

### Running the Nodes

1. **Using launch file** (recommended):
   ```bash
   # Parameters are loaded from config files in the install directory
   ros2 launch ros2_wave_pkg wave_nodes.launch.py
   ```

2. **Running nodes separately**:
   ```bash
   # Terminal 1 - Start subscriber
   ros2 run ros2_wave_pkg sine_wave_subscriber
   
   # Terminal 2 - Start publisher
   ros2 run ros2_wave_pkg sine_wave_publisher
   ```

   Note: When running separately, nodes use default parameter values set using module generated by generate_parameter_library unless specified otherwise.

### Parameter Configuration

#### Available Parameters

1. **Publisher Parameters**:
   ```yaml
   sine_wave_publisher:
     publisher_frequency: # [0.0-100.0 Hz] read-only
     amplitude:          # [0.0-50.0]
     angular_frequency:  # [0.0-62.8 rad/s] (2π rad/s = 1 Hz)
     phase:             # [-6.28-6.28 rad]
   ```

2. **Subscriber Parameters**:
   ```yaml
   sine_wave_subscriber:
     buffer_size:    # [250-1500] read-only
     enable_plot:    # [true/false] read-only
   ```

#### Configuration Methods

1. **Edit YAML and rebuild** (Recommended):
   ```bash
   # Edit source files
   cd ~/ros2_ws/src/ros2-dev-challenge/ros2_wave_pkg/config
   # Edit sine_wave_publisher_params.yaml or sine_wave_subscriber_params.yaml
   
   # Rebuild package
   cd ~/ros2_ws
   colcon build --packages-select ros2_wave_pkg
   source install/setup.bash
   ```

2. **Edit in install directory** (Quick but not ideal approach):
   ```bash
   cd ~/ros2_ws/install/ros2_wave_pkg/share/ros2_wave_pkg/config
   # Edit configuration files sine_wave_publisher_params.yaml or sine_wave_subscriber_params.yaml directly
   ```

3. **Dynamic parameter updates** (Runtime, non read-only only):
   ```bash
   # List all parameters
   ros2 param list
   
   # Set parameters
   ros2 param set /sine_wave_publisher amplitude 5.0
   ros2 param set /sine_wave_publisher angular_frequency 3.14
   ros2 param set /sine_wave_publisher phase -3.14
   
   # Get parameter information
   ros2 param describe /sine_wave_publisher amplitude
   ```

   Example responses:
   ```plaintext
   - Set parameter successful
   - Setting parameter failed: Wrong parameter type
   - Setting parameter failed: Value out of bounds
   - Setting parameter failed: Trying to set read-only parameter
   ```

### Visualization Control

Control real-time plotting in two ways:

1. **Using parameter file**:
   ```yaml
   # Edit ~/ros2_ws/install/ros2_wave_pkg/share/ros2_wave_pkg/config/sine_wave_subscriber_params.yaml
   sine_wave_subscriber:
     ros__parameters:
       enable_plot: false
   ```

2. **Using command line**:
   ```bash
   ros2 run ros2_wave_pkg sine_wave_subscriber --ros-args -p enable_plot:=false
   ```

### Image Processing Service

The package provides a service to convert color images to grayscale with visualization options:

1. **Using custom image**:
   ```bash
   ros2 service call /process_image custom_interfaces/srv/ProcessImage "{
     image_path: '/path/to/your/image.jpg',
     show_visualization: true
   }"
   ```

2. **Using built-in test image**:
   ```bash
   ros2 service call /process_image custom_interfaces/srv/ProcessImage "{
     image_path: 'test',
     show_visualization: true
   }"
   ```

Parameters:
- `image_path`: Path to input image (use 'test' for built-in test image)
- `show_visualization`: When true, displays side-by-side comparison

## Development

### Testing
```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select ros2_wave_pkg

# Run specific test
colcon test --packages-select ros2_wave_pkg --pytest-args -k "test_nodes_comms_and_props"

# View detailed results
colcon test-result --verbose
```

### Code Quality
```bash
# Run all linting checks
cd ~/ros2_ws/src/ros2-dev-challenge
chmod +x run_linting.sh
./run_linting.sh

# Individual checks
flake8 ros2_wave_pkg -v
black ros2_wave_pkg --check --diff -v
isort ros2_wave_pkg --check-only --diff -v
```

## Docker Commands Reference

### Basic Operations
```bash
# Image Management
docker images                     # List images
docker rmi ros2_wave_pkg         # Remove specific image
docker image prune               # Remove unused images

# Container Management
docker ps                        # List running containers
docker ps -a                     # List all containers
docker stop <container_id>       # Stop container
docker rm <container_id>         # Remove container
docker container prune           # Remove stopped containers
```

### Advanced Operations
```bash
# Container Interaction
docker exec -it <container_id> bash  # Open new terminal in container. you need to open new cmd tab and then execute "docker ps" and take the container id and then use that id in this cmd.
docker cp <container_id>:/src /dest  # Copy from container to host
docker cp /src <container_id>:/dest  # Copy from host to container

# System Cleanup
docker container prune           # Remove stopped containers
docker image prune              # Remove unused images
docker volume prune             # Remove unused volumes
docker system prune -a          # Remove all unused resources
```

## License
This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Support
For issues, questions, or feature requests:
- Open an issue on the GitHub repository
- Provide detailed information about your environment and problem
- Include relevant logs and error messages