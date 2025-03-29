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
	udev \
	neovim \
	libxcb-xinerama0 \
	libxcb-cursor0 \
	libgl1-mesa-glx \
	libgl1-mesa-dev \
	python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install RPi.GPIO

# Set up workspace
WORKDIR /ros2_ws

COPY src /ros2_ws/src/

RUN cd src \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

# Build the workspace
RUN . /opt/ros/humble/setup.sh \
    && cd /ros2_ws \
    && colcon build --symlink-install

# Source ROS 2 and workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN chmod +x src/ydlidar_ros2_driver-master/startup/*
RUN sudo sh src/ydlidar_ros2_driver-master/startup/initenv.sh

RUN mv src/Makefile .

COPY ./params/ydlidar_x3.yaml src/ydlidar_ros2_driver-master/params

# Default command: start a bash shell
CMD ["/bin/bash", "-c", "while true; do sleep 3600; done"]
