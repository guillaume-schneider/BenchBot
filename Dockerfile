# SLAM Bench Orchestrator - Docker Environment

# Use official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt5 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    libgl1-mesa-glx \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
RUN pip3 install \
    pyyaml \
    numpy \
    matplotlib \
    reportlab \
    psutil \
    transforms3d

# Setup workspace
WORKDIR /app
COPY . .

# Build GMapping workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd deps/gmapping_ws && \
    colcon build --symlink-install"

# Default command (Headless CI runner)
# To run GUI, you'll need to share X11 socket
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source deps/gmapping_ws/install/setup.bash && exec \"$@\""]
CMD ["python3", "runner/run_matrix.py", "configs/matrices/test_headless_ci.yaml"]
