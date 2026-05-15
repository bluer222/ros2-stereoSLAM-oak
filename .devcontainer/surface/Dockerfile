FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ARG DEBIAN_FRONTEND=noninteractive

# ROS 2 Jazzy install on Ubuntu 24.04
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    wget \
    ca-certificates \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rviz2 \
    ros-jazzy-stereo-image-proc \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*
# onnxruntime-gpu built against CUDA 12.x — no separate CUDA install needed,
# it's already in the base image
RUN pip install onnxruntime-gpu==1.20.1 --break-system-packages

# Install YOLOv8 and dependencies for crab detector
RUN pip install --no-cache-dir --break-system-packages \
    'ultralytics>=8.0.0' \
    torch \
    torchvision \
    keyboard

RUN rosdep init || true

WORKDIR /workspace
ENV PATH="/opt/ros/jazzy/bin:${PATH}"
#ENV ROS_DOMAIN_ID=0
#ENV ROS_IP=192.168.3.2
#ENV ROS_HOSTNAME=192.168.3.2
#ENV ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
#ENV ROS_STATIC_PEERS="192.168.3.1;192.168.2.2"
#ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
#ENV RMW_FASTRTPS_USE_QOS_FROM_XML=1
#ENV FASTDDS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"
#ENV FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
