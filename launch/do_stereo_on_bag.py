from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    bag_path = '/workspace/oak_session/20260418_023007/rosbag'
    output_bag_path = '/workspace/oak_session/20260418_023007/stereo_output'

    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',  # publish /clock so nodes use bag time
        ],
        output='screen'
    )

    record_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', output_bag_path,
            '/stereo/depth/image_rect',
            '/stereo/depth/camera_info',
        ],
        output='screen'
    )

    stereo_processor = Node(
        package='stereo_processor',
        executable='stereo_processor',
        name='stereo_processor',
        parameters=[{
            'left_rect_topic': '/oak/left/image_raw',
            'right_rect_topic': '/oak/right/image_raw',
            'left_info_topic': '/oak/left/camera_info',
            'rgb_info_topic': '/oak/rgb/camera_info',
            'depth_topic': '/stereo/depth/image_rect',
            'depth_info_topic': '/stereo/depth/camera_info',
            'disparity_topic': '/stereo/disparity',
            'model_path': '/workspace/ai/crestereo_iter5.onnx',
        }],
        output='screen',
    )

    return LaunchDescription([
        record_bag,
        stereo_processor,
        TimerAction(period=2.0, actions=[play_bag]),  # let recorder + node spin up first
    ])