FROM ros:jazzy-ros-base

# Noninteractive apt
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    ca-certificates \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    lsb-release \
    libopencv-dev \
    python3-opencv \
    pkg-config \
    tini \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
# Initialize rosdep (container image at build-time)
RUN rosdep init || true

WORKDIR /workspace

ENV PATH="/opt/ros/jazzy/bin:${PATH}"
ENV ROS_DOMAIN_ID=0
ENV ROS_IP=192.168.3.1
ENV ROS_HOSTNAME=192.168.3.1
ENV ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
ENV RMW_FASTRTPS_USE_QOS_FROM_XML=1
ENV FASTDDS_DEFAULT_PROFILES_FILE="/workspace/fastdds/pi_usb.xml"
ENV FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/fastdds/pi_usb.xml"

# Ensure interactive shells source the system ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Copy source code (just streaming package)
COPY src/zed_streamer /workspace/src/zed_streamer
#copy fastdds configuration files
COPY fastdds/pi_usb.xml /workspace/fastdds/pi_usb.xml
COPY fastdds/pi_eth.xml /workspace/fastdds/pi_eth.xml


# Copy entrypoint script
COPY .devcontainer/pi/setuppi.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# BlueOS Extension Metadata Labels
LABEL version="1.0.0-beta.1"
LABEL authors='[\
    {\
        "name": "Your Name",\
        "email": "your.email@example.com"\
    }\
]'
LABEL company='{\
    "about": "Stereo vision processing extension for BlueOS",\
    "name": "Your Organization",\
    "email": "support@example.com"\
}'
LABEL readme="https://raw.githubusercontent.com/yourusername/mapping4/{tag}/README.md"
LABEL links='{\
    "github": "https://github.com/yourusername/mapping4",\
    "documentation": "https://github.com/yourusername/mapping4/wiki"\
}'
LABEL type="device-integration"
LABEL tags="mapping,ros2,stereo-vision,camera"
LABEL permissions='{\
    "HostConfig": {\
        "NetworkMode": "host",\
        "Binds": [\
            "/dev:/dev",\
            "/dev/v4l:/dev/v4l",\
            "/dev/bus/usb:/dev/bus/usb",\
            "/run/udev:/run/udev:ro",\
            "/usr/blueos/extensions/stereo-processor:/app/data"\
        ],\
        "Privileged": true\
    }\
}'

#permissions without the backslash
#LABEL permissions2='{ "HostConfig": { "NetworkMode": "host", "Binds": [ "/dev:/dev", "/dev/v4l:/dev/v4l", "/dev/bus/usb:/dev/bus/usb", "/run/udev:/run/udev:ro", "/usr/blueos/extensions/stereo-processor:/app/data" ], "Privileged": true } }'


ENTRYPOINT ["/bin/bash"]
CMD ["/entrypoint.sh"]
