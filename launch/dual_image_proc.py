from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ------------------------------------------------------------------ #
    #  stereo_camera_node
    #  Publishes on:
    #    /stereo/left/image_rect
    #    /stereo/left/camera_info
    #    /stereo/right/image_rect
    #    /stereo/right/camera_info
    # ------------------------------------------------------------------ #
    stereo_camera = Node(
        package='post_util',        # <-- replace with your package
        executable='dual',
        name='dual',
        output='screen',
        parameters=[{
            'input_topic':      '/camera/dual/image_raw',
            'calibration_file': '/workspace/SN15095.conf',
            'resolution':       'HD',
            'image_width':      1280,
            'image_height':     720,
        }],
        #no remapigs neseccary, swaped default to /stereo
    )

    # ------------------------------------------------------------------ #
    #  stereo_image_proc  (runs as a composable node for zero-copy)
    #
    #  Subscribes to (via remapping):
    #    left/image_rect_color  or  left/image_rect  (mono)
    #    left/camera_info
    #    right/image_rect_color or  right/image_rect (mono)
    #    right/camera_info
    #
    #  Publishes (among others):
    #    /stereo/disparity
    #    /stereo/points2
    # ------------------------------------------------------------------ #
    stereo_proc_container = ComposableNodeContainer(
        name='stereo_proc_container',
        namespace='stereo',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                namespace='stereo',
                parameters=[{
                    'approximate_sync': False,
                    # SGM / BM tuning — adjust to taste
                    'stereo_algorithm':     1,      # 0=BM, 1=SGM
                    'min_disparity':        0,
                    'disparity_range':      256,
                    'uniqueness_ratio':     0.0,
                    'block_size':        3,        # must be odd, 3-11
                    'speckle_size':         1000,
                    'speckle_range':        7,
                }],
                remappings=[
                    # stereo_image_proc expects siblings relative to namespace
                    ('left/image_rect',  '/stereo/left/image_rect'),
                    ('left/camera_info',       '/stereo/left/camera_info'),
                    ('right/image_rect', '/stereo/right/image_rect'),
                    ('right/camera_info',      '/stereo/right/camera_info'),
                    ('disparity',              '/stereo/disparity'),
                ],
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                namespace='stereo',
                parameters=[{
                    'approximate_sync': False,
                    'use_color':        False,      # set True if publishing bgr8
                    'queue_size':       10,
                }],
                remappings=[
                    ('left/image_rect',  '/stereo/left/image_rect'),
                    ('left/camera_info',       '/stereo/left/camera_info'),
                    ('right/camera_info',      '/stereo/right/camera_info'),
                    ('disparity',              '/stereo/disparity'),
                    ('points2',                '/stereo/points2'),
                ],
            ),
        ],
        output='screen',
    )

    #publishes dual image raw
    streamer = Node(
        package='zed_streamer',
        executable='opencv_streamer',
        name='opencv_streamer',
        output='screen'
    )
    

    return LaunchDescription([stereo_camera, stereo_proc_container, streamer])