#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get package share directory
    pkg_share = get_package_share_directory('l2i_fusion_detection')

    # RViz2 with proper path resolution
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'l2i.rviz')
    
    # Fallback to a generic config if specific one doesn't exist
    if not os.path.exists(rviz_config_path):
        rviz_config_path = os.path.join(pkg_share, 'l2i.rviz')

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]

    )
    
    # topics if using gazebo simulation 
    image_topic = '/camera_1/image_raw'
    lidar_topic = '/ray/lidar_points'
    camera_info_topic = '/camera_1/camera_info'
    camera_frame = 'camera_1_link'
    lidar_frame = 'lidar_link'
    ground_height_threshold = 0.66
    
    # # use recorded rosbag topics
    # image_topic = '/image_raw'
    # lidar_topic = '/rslidar_points'
    # camera_info_topic = '/camera_info'
    # camera_frame = 'camera_link'
    # lidar_frame = 'lidar_link'
    # ground_height_threshold = 0.05
    
    # Standard YOLO Launch for Lidar-Camera Fusion
    yolo_launch_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch/yolo.launch.py'
            ])
        ]),
        launch_arguments={
            'model': "yolov8m-seg.pt",
            'threshold': '0.1',
            'input_image_topic': image_topic,
            'namespace': 'yolo',
            'device': 'cuda:0'
        }.items()
    )
    
    tracking_topic = '/yolo/tracking' # this is the output topic from the standard yolo launch file
    
    # Lidar-Camera Fusion Node
    '''
    This node requires valid transform between the lidar frame and camera frame.
    Make sure this is already being published in simulation env/ on real robot
    '''
    lidar_camera_fusion_node = Node(
        package='l2i_fusion_detection',
        executable='lidar_camera_fusion_with_detection',
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_range': 0.5, 'max_range': 30.0,
             'lidar_frame': lidar_frame,
             'camera_frame': camera_frame,
             'use_sim_time': True,
             'enable_ground_filtering': True,
             'ground_height_threshold': ground_height_threshold
             }
        ],
        remappings=[
            ('/lidar_points', lidar_topic),
            ('/camera/camera_info', camera_info_topic),
            ('/camera/image_raw', image_topic),
            ('/yolo/tracking', tracking_topic)
        ]
    )

    # Add all nodes and launches to the launch description
    ld.add_action(rviz_node)
    ld.add_action(lidar_camera_fusion_node)
    ld.add_action(yolo_launch_fusion)

    return ld
