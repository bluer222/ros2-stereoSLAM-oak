#!/bin/bash
#This script will run on a pi using BlueOS and stream the camera feed

set -e

echo "=========================================="
echo "Running setup script for ROS 2 Jazzy"
echo "=========================================="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"


cd /workspace || exit 1

# Only do installation steps on first run
#do this by chedcking if local setup.bash exists, which is only generated after the first build
if [ -f ./install/local_setup.bash ]; then
    echo "Workspace already built, skipping installation steps"
else
    echo "First run - installing dependencies and building workspace..."
    
    # Initialize rosdep if needed
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
            echo "Initializing rosdep..."
            rosdep init || true
    fi

    echo "Updating apt lists and rosdep"
    apt-get update || true
    rosdep update || true

    echo "Installing package dependencies from package.xml / package.yaml"
    rosdep install --from-paths src --ignore-src -r -y || true

    echo "Building the workspace with colcon"
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
fi

echo "Setting up FastDDS environment"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTDDS_DEFAULT_PROFILES_FILE="/workspace/fastdds/pi_usb.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/fastdds/pi_usb.xml"
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"

echo "Sourcing ROS 2 Jazzy base setup"
source /opt/ros/jazzy/setup.bash

echo "Sourcing workspace overlay"
if [ -f ./install/local_setup.bash ]; then
    source ./install/local_setup.bash
fi

echo "Restarting ROS 2 daemon to apply FastDDS configuration"
ros2 daemon stop || true
ros2 daemon start || true

echo "=========================================="
echo "FastDDS configuration complete"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "FastDDS Config: $FASTDDS_DEFAULT_PROFILES_FILE"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "=========================================="
echo "Starting ROS2 node"
echo "=========================================="

# Run the node continuously (replaces this shell process)
exec ros2 run zed_streamer opencv_streamer
