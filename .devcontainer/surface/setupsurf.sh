#!/bin/bash
# postCreateCommand: runs once after the devcontainer is created.
# CUDA + cuDNN + onnxruntime-gpu are already in the image — this script
# only handles workspace-level setup.

set -e
echo "Running setup script for ROS 2 Jazzy (workspace: /workspace)"

cd /workspace || exit 1

# Initialize rosdep if needed (image does rosdep init at build time,
# but the sources list may not exist on a fresh volume mount)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || true
fi

apt-get update -qq || true
rosdep update || true

echo "Installing ROS package dependencies"
rosdep install --from-paths src --ignore-src -r -y || true

echo "Building workspace"
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

#install depthai sdk for ros2
apt install ros2-testing-apt-source -qq -y || true
apt update -qq || true
apt install ros-$ROS_DISTRO-depthai-ros-v3 -qq -y || true

# Install YOLOv8 for crab detector
pip install --break-system-packages -q torch torchvision 'ultralytics>=8.0.0' keyboard || true

echo "Sourcing workspace"
if [ -f ./install/local_setup.bash ]; then
    source ./install/local_setup.bash
    #copy to bashrc because i'm lazy
    echo "source /workspace/install/local_setup.bash" >> /root/.bashrc
fi

# Configure FastDDS environment for interactive shells
echo "Configuring FastDDS for ROS2 communication"
cat >> /root/.bashrc << 'EOF'

# FastDDS RMW Configuration
#export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
#export RMW_FASTRTPS_USE_QOS_FROM_XML=1
#export FASTDDS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"
#export FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"
#export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"

# After setting the ENV, you need to stop and restart the ROS2 daemon.
# ros2 daemon stop
# ros2 daemon start
EOF

# Export variables for current session before restarting daemon
#export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
#export RMW_FASTRTPS_USE_QOS_FROM_XML=1
#export FASTDDS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"
#export FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/fastdds/surface_usb.xml"
#export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"

echo "Restarting ROS 2 daemon to apply FastDDS configuration"
ros2 daemon stop || true
ros2 daemon start || true

echo "Setup complete."
