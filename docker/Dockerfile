FROM ros:iron-ros-base

# Install required packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    ros-iron-pcl-conversions \
    ros-iron-pcl-ros \
    ros-iron-rmw-fastrtps-cpp \
    ros-iron-sensor-msgs \
    ros-iron-rosbag2-* \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Source ROS on container start
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
