#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crab_detector',
            executable='crab_detector_node',
            name='crab_detector',
            output='screen',
            parameters=[
            ]
        ),
    ])
