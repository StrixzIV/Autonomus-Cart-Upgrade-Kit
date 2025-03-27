FROM ros:humble-ros-base

# Install necessary dependencies
RUN apt update && apt install -y \
	pkg-config \
	swig \
	python3-pip \
    python3-colcon-common-extensions \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws

RUN mkdir -p src

RUN cd src \
    && git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN cd src \
    && git clone https://github.com/Oyefusi-Samuel/ydlidar_ros2_driver-master.git

# # Build the workspace
RUN . /opt/ros/humble/setup.sh \
    && cd /ros2_ws \
    && colcon build --symlink-install

# Source ROS 2 and workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command: start a bash shell
CMD ["/bin/bash", "-c", "while true; do sleep 3600; done"]
