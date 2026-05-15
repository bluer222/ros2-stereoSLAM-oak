import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    depthai_launch_dir = os.path.join(
        get_package_share_directory('depthai_ros_driver_v3'), 
        'launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_launch_dir, 'driver.launch.py')
            ),
            # This acts like the "params_file:=" part of your terminal command
            launch_arguments={
                'params_file': '/workspace/oak.yaml'
            }.items()
        ),
        Node(
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
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            namespace='rtabmap',
            name='rgbd_odometry',
            parameters=[{
                'frame_id': 'oak_rgb_camera_optical_frame',
                'odom_frame_id': 'odom',
                'approx_sync': True,
                'topic_queue_size': 50,
                'sync_queue_size': 25,
                'wait_for_transform': 0.2,
                'qos': 1,
                'qos_image': 1,
                'Odom/Strategy': '0',
                'Vis/MinInliers': '12',
                'Vis/FeatureType': '8',
                'Vis/CorFlowMaxLevel': '3',
                'Reg/Strategy': '0',
                'Odom/ResetCountdown': '50',
                'Odom/MaxFeatures': '0',
            }],
            remappings=[
                ('rgb/image', '/oak/rgb/image_raw'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/stereo/depth/image_rect'),
                ('depth/camera_info', '/stereo/depth/camera_info'),
                ('odom', 'odom'),
                ('odom_info', 'odom_info'),
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='rtabmap',
            name='rtabmap',
            parameters=[{
                'database_path': '/tmp/rtabmap.db',
                'frame_id': 'oak_rgb_camera_optical_frame',
                'approx_sync': True,
                'topic_queue_size': 50,
                'sync_queue_size': 25,
                'qos': 1,
                'qos_image': 1,
                'qos_imu': 1,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': True,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/STMSize': '50',
                'Rtabmap/DetectionRate': '1',
                'Mem/RehearsalSimilarity': '0.20',
                'Kp/MaxFeatures': '600',
                'Kp/DetectorStrategy': '8',
                'Vis/MinInliers': '15',
                'Vis/InlierDistance': '0.1',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '50',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.02',
                'RGBD/LinearUpdate': '0.02',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/OptimizeMaxError': '5.0',
                'Optimizer/Robust': 'true',    # Vertigo robust optimization, downweights bad closures
                'Optimizer/Strategy': '2',
                'GTSAM/Incremental': 'true',
                'Optimizer/Iterations': '100',
                'Reg/Strategy': '1',
                'Reg/Force3DoF': 'false',
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.3',
                'Icp/CorrespondenceRatio': '0.15',
                'Icp/Iterations': '50',
                'Icp/PointToPlane': 'true',
                'Grid/RangeMax': '8.0',
                'Grid/RangeMin': '0.3',
                'Grid/CellSize': '0.05',
                'Grid/3D': 'true',
                'Grid/FromDepth': 'true',
                'Rtabmap/TimeThr': '0',
                'Rtabmap/MemoryThr': '0',
                'Mem/BadSignaturesIgnored': 'false',
                'Mem/RehearsalIdUpdatedToNewOne': 'true',
            }],
            remappings=[
                ('rgb/image', '/oak/rgb/image_raw'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/stereo/depth/image_rect'),
                ('depth/camera_info', '/stereo/depth/camera_info'),
                ('odom', 'odom'),
                ('odom_info', 'odom_info'),
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/workspace/launch/oak_stereo_rtabmap.rviz'],
            output='screen',
        ),
    ])
