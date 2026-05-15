from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # 4. OpenCV Streamer
    Node(
        package='zed_streamer',
        executable='opencv_streamer',
        name='opencv_streamer',
        output='screen'
    )

    stereo_camera = Node(
        package='post_util',    # <-- replace with your package name
        executable='dual',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            # Raw side-by-side topic from the camera driver
            'input_topic': '/camera/dual/image_raw',

            # Absolute path to your ZED .conf calibration file
            'calibration_file': '/workspace/SN15095.conf',

            # Must match the section headers in the .conf file.
            # Common values: 'HD' (1280x720), 'FHD' (1920x1080), 'VGA' (672x376)
            'resolution': 'HD',

            # Must match the resolution above
            'image_width':  1280,
            'image_height': 720,
        }],
        remappings=[
            ('/camera/left/image_rect',   '/stereo/left/image_rect'),
            ('/camera/right/image_rect',  '/stereo/right/image_rect'),
            ('/camera/left/camera_info',  '/stereo/left/camera_info'),
            ('/camera/right/camera_info', '/stereo/right/camera_info'),
        ],
    )

    ess_container = ComposableNodeContainer(
        name='ess_container',
        namespace='stereo',
        package='rclcpp_components',
        executable='component_container_mt',    # must be multi-threaded for NITROS/ESS
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_ess',
                plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
                name='ess_disparity',
                namespace='stereo',
                parameters=[{
                    # Absolute path to the TensorRT engine plan.
                    # Download from NGC: models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx_trt10.13/
                    # Two options:
                    #   ess.engine       — full model, higher accuracy, slower
                    #   light_ess.engine — lighter model, faster, slightly lower accuracy
                    'engine_file_path': '/workspace/light_ess.onnx',

                    # Confidence threshold [0.0, 1.0].
                    # Pixels with confidence below this are set to -1.0 (invalid).
                    # 0.0  = fully dense output, no filtering
                    # 0.35 = NVIDIA's recommended default for most use cases
                    # 0.5+ = sparser but higher confidence, good for mapping/navigation
                    'threshold': 0.35,
                }],
                remappings=[
                    ('left/image_rect',   '/stereo/left/image_rect'),
                    ('left/camera_info',  '/stereo/left/camera_info'),
                    ('right/image_rect',  '/stereo/right/image_rect'),
                    ('right/camera_info', '/stereo/right/camera_info'),
                    ('disparity',         '/stereo/disparity'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([stereo_camera, ess_container])