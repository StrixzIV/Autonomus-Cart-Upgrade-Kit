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

COPY cmake-config /ros2_ws/cmake-config

RUN mkdir -p src

RUN git clone --branch master https://github.com/YDLIDAR/sdk.git src/ydlidar_sdk

RUN . /opt/ros/humble/setup.sh && \
    cd src/ydlidar_sdk && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4
	# sudo make install
# 	make -j4 && \
# 	cp -r ../include/* /usr/local/include && \
# 	cp libydlidar_driver.a /usr/local/lib/ && \
#     ldconfig

# ENV CMAKE_PREFIX_PATH="/ros2_ws/cmake-config:$CMAKE_PREFIX_PATH"
# ENV CMAKE_PREFIX_PATH="/ros2_ws/src/ydlidar_sdk:$CMAKE_PREFIX_PATH"

RUN cd src \
    && git clone https://github.com/Oyefusi-Samuel/ydlidar_ros2_driver-master.git

# RUN cp -r /ros2_ws/src/ydlidar_sdk/include/* /ros2_ws/src/ydlidar_ros2_driver-master/src

# # Build the workspace
# RUN . /opt/ros/humble/setup.sh \
#     && cd /ros2_ws \
#     && colcon build --symlink-install

# Source ROS 2 and workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command: start a bash shell
CMD ["/bin/bash", "-c", "while true; do sleep 3600; done"]
