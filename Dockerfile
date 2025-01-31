# Use ROS2 Humble as base image
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies with specific versions
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv=4.5.4+dfsg-9ubuntu4 \
    python3-matplotlib=3.5.1-2build1 \
    python3-numpy=1:1.21.5-1ubuntu22.04.1 \
    ros-humble-rclpy=3.3.15-1jammy.20241128.012616 \
    ros-humble-std-msgs=4.2.4-1jammy.20241128.004339 \
    ros-humble-sensor-msgs=4.2.4-1jammy.20241128.010813 \
    ros-humble-sensor-msgs-py=4.2.4-1jammy.20241128.024210 \
    ros-humble-cv-bridge \
    git \
    && rm -rf /var/lib/apt/lists/*


# Create workspace structure
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Clone generate_parameter_library first
RUN git clone https://github.com/picknikrobotics/generate_parameter_library.git

# Clone your repository with specific branch
RUN git clone -b develop https://github.com/Abhi-0212000/ros2-dev-challenge.git

# Set working directory to workspace root
WORKDIR /ros2_ws

# Update rosdep and install dependencies
RUN apt-get update && \
    rosdep update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    # First build generate_parameter_library
    colcon build --packages-select generate_parameter_library generate_parameter_library_py parameter_traits && \
    . install/setup.sh && \
    # Then build custom_interfaces
    colcon build --packages-select custom_interfaces && \
    . install/setup.sh && \
    # Then build ros2_wave_pkg
    colcon build --packages-select ros2_wave_pkg

# Add convenient aliases for sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# # Create entrypoint script directly in the container
# RUN echo '#!/bin/bash\n\
# set -e\n\
# \n\
# # Source ROS 2 setup\n\
# source "/opt/ros/$ROS_DISTRO/setup.bash"\n\
# source "/ros2_ws/install/setup.bash"\n\
# \n\
# exec "$@"' > /ros_entrypoint.sh && \
#     chmod +x /ros_entrypoint.sh

# Create an entrypoint script
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]