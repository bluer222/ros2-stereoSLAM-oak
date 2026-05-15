import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    depthai_launch_dir = os.path.join(
        get_package_share_directory('depthai_ros_driver_v3'),
        'launch'
    )

    return LaunchDescription([

        # OAK camera driver ONLY
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_launch_dir, 'driver.launch.py')
            ),
            launch_arguments={
                'params_file': '/workspace/oak.yaml'
            }.items()
        ),

        # Rosbag recording
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',

                # Topics (ONLY what you actually need)
                '/oak/left/image_raw',
                '/oak/right/image_raw',
                '/oak/rgb/image_raw',

                '/oak/left/camera_info',
                '/oak/right/camera_info',
                '/oak/rgb/camera_info',

                # High cache (RAM buffer)
                '--max-cache-size', '10737418240',  # 10 GB

                # Optional: better storage backend
                '--storage', 'mcap'
            ],
            output='screen'
        )
    ])