FROM osrf/ros:foxy

# Install necessary dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-foxy-slam-toolbox \
    ros-foxy-rviz2 \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
RUN mkdir -p src && cd src \
    && git clone https://github.com/Oyefusi-Samuel/ydlidar_ros2_driver-master.git

# Build the workspace
RUN . /opt/ros/foxy/setup.sh \
    && cd /ros2_ws \
    && colcon build --symlink-install

# Source ROS 2 and workspace automatically
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command: start a bash shell
CMD ["/bin/bash"]