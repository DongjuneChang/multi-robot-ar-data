# Multi-stage build: Get v0.7.2 from husarion image for Lichtblick compatibility
FROM husarion/foxglove-bridge:humble AS foxglove-bridge-072

# Main build stage
FROM robot-base:humble

# Set up Arizona mirror for APT
RUN echo "deb http://mirror.arizona.edu/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list

WORKDIR /workspace

# Clone xarm_ros2 repo with submodules
RUN cd /workspace/src && \
    git clone --recursive https://github.com/xArm-Developer/xarm_ros2.git -b humble && \
    cd xarm_ros2 && \
    git submodule sync && \
    git submodule update --init --remote


# Install additional dependencies for xarm5 and network tools
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install -y net-tools && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Build xarm_ros2 packages with file copy instead of symlinks
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    # Build uf_ros_lib first (dependency for xarm_moveit_config)
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select uf_ros_lib && \
    # Build xarm_msgs
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_msgs && \
    # Build xarm_sdk
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_sdk && \
    # Build xarm_description
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_description && \
    # Build xarm_api with reduced optimization
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-O1' --packages-select xarm_api && \
    # Build xarm_controller and xarm_gazebo (dependencies for xarm_moveit_config)
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_controller xarm_gazebo && \
    # Now build xarm_moveit_config
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_moveit_config && \
    # Build remaining xarm packages
    colcon build --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DCMAKE_BUILD_TYPE=Release --packages-select xarm_moveit_servo xarm_planner && \
    echo 'xarm_ros2 packages built successfully'

# Install FlexBE packages from APT (more stable than building from source)
RUN apt-get update && \
    apt-get install -y \
    ros-humble-flexbe-core \
    ros-humble-flexbe-msgs \
    ros-humble-flexbe-onboard \
    ros-humble-flexbe-states \
    ros-humble-flexbe-behavior-engine && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Foxglove Bridge for ROS2 visualization
# Install apt version first (for dependencies), then replace with v0.7.2 for Lichtblick compatibility
# v0.7.2 uses foxglove.websocket.v1 protocol (compatible with Lichtblick)
# v3.2.2 uses foxglove.sdk.v1 protocol (not compatible)
RUN apt-get update && \
    apt-get install -y ros-humble-foxglove-bridge && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy v0.7.2 from husarion image (Lichtblick compatible)
# Replace apt-installed v3.2.2 with v0.7.2 for foxglove.websocket.v1 protocol support
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/foxglove_bridge/foxglove_bridge /opt/ros/humble/lib/foxglove_bridge/foxglove_bridge
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/libfoxglove_bridge_base.so /opt/ros/humble/lib/libfoxglove_bridge_base.so
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/libfoxglove_bridge_component.so /opt/ros/humble/lib/libfoxglove_bridge_component.so

# Clone and build MoveIt Task Constructor
RUN cd /workspace/src && \
    git clone --recursive https://github.com/moveit/moveit_task_constructor.git -b humble && \
    cd /workspace && \
    . /opt/ros/humble/setup.sh && \
    # Install dependencies first
    apt-get update && \
    rosdep install --from-paths src/moveit_task_constructor --ignore-src -r -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    # Build MTC packages
    colcon build --symlink-install --packages-select \
    moveit_task_constructor_msgs \
    rviz_marker_tools \
    moveit_task_constructor_core \
    moveit_task_constructor_capabilities \
    moveit_task_constructor_visualization \
    moveit_task_constructor_demo \
    --cmake-args -DCMAKE_BUILD_TYPE=Release



# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

WORKDIR /workspace

# Build the trac_ik_lib package
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    colcon build --symlink-install --packages-select trac_ik_lib --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["tail", "-f", "/dev/null"]
