# Use ROS2 Humble as base image
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install git (needed for cloning repositories)
RUN apt-get update && \
    apt-get install -y git python3-pip && \
    rm -rf /var/lib/apt/lists/*

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
    # First build generate_parameter_library pkgs that we need
    colcon build --packages-select generate_parameter_library generate_parameter_library_py parameter_traits && \
    . install/setup.sh && \
    # Then build custom_interfaces, ros2_wave_pkg
    colcon build --packages-select custom_interfaces ros2_wave_pkg && \
    . install/setup.sh

# Add convenient aliases for sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# copy and execute entrypoint script
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]