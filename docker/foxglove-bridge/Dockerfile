# Foxglove Bridge Dockerfile
# Provides WebSocket bridge between ROS2 and web panel
FROM ros:humble-ros-base

# Install Foxglove Bridge
RUN apt-get update && \
    apt-get install -y ros-humble-foxglove-bridge && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Default port
EXPOSE 8765

# Entrypoint
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"]
