from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 4. OpenCV Streamer
        Node(
            package='zed_streamer',
            executable='opencv_streamer',
            name='opencv_streamer',
            output='screen'
        )
    ])
