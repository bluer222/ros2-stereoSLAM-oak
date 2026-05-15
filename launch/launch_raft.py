from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Stereo Processor
        Node(
            package='stereo_processor',
            executable='raft',
            name='stereo_processor',
            parameters=[{
                #'calibration_file': '/workspace/SN15095.conf',
                'model_path': '/workspace/raftstereo_realtime.onnx'
            }],
            output='screen'
        ),

        # 2. Frame Rectifier
        Node(
            package='post_util',
            executable='dual',
            name='split_rectifier',
            parameters=[{
                'calibration_file': '/workspace/SN15095.conf'
            }],
            output='screen'
        ),

        # 4. OpenCV Streamer
        Node(
            package='zed_streamer',
            executable='opencv_streamer',
            name='opencv_streamer',
            output='screen'
        )
    ])
