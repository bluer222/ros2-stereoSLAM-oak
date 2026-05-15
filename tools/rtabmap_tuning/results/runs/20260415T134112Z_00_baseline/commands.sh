#!/usr/bin/env bash
ros2 run rtabmap_odom rgbd_odometry --ros-args \
  -r __ns:=/rtabmap \
  -r __node:=rgbd_odometry \
  --params-file "/workspace/tools/rtabmap_tuning/results/runs/20260415T134112Z_00_baseline/preset.yaml" \
  -r rgb/image:=/stereo/left/image_rect \
  -r rgb/camera_info:=/stereo/left/camera_info \
  -r depth/image:=/stereo/depth/image_rect \
  -r depth/camera_info:=/stereo/left/camera_info \
  -r odom:=odom

ros2 run rtabmap_slam rtabmap --ros-args \
  -r __ns:=/rtabmap \
  -r __node:=rtabmap \
  --params-file "/workspace/tools/rtabmap_tuning/results/runs/20260415T134112Z_00_baseline/preset.yaml" \
  -p database_path:="/workspace/tools/rtabmap_tuning/results/runs/20260415T134112Z_00_baseline/rtabmap.db" \
  -r rgb/image:=/stereo/left/image_rect \
  -r rgb/camera_info:=/stereo/left/camera_info \
  -r depth/image:=/stereo/depth/image_rect \
  -r depth/camera_info:=/stereo/left/camera_info \
  -r odom:=/rtabmap/odom

ros2 bag play "./tools/rtabmap_tuning/bags/library" --clock --rate "1.0" 
