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

    tracking_topic = '/yolo/tracking' # for standard yolo model
    
    # using gazebo simulation topics
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
    
    # Lidar-Camera Fusion Node
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
            ('/scan/points', lidar_topic),
            ('/observer/gimbal_camera_info', camera_info_topic),
            ('/observer/gimbal_camera', image_topic),
            ('/rgb/tracking', tracking_topic)
        ]
    )

    # YOLO Model path using rgb model from origina repo
    # model_path = os.path.join(pkg_share, 'config', 'rgb.pt')
    # # YOLO Launch for Lidar-Camera Fusion
    # yolo_launch_fusion = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('yolo_bringup'),
    #             'launch/yolov11.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'model': model_path,
    #         # 'model': 'yolo11n.pt',
    #         'threshold': '0.2',
    #         'input_image_topic': image_topic,
    #         'namespace': 'rgb',
    #         'device': 'cuda:0'
    #     }.items()
    # )

    # standard YOLO Launch for Lidar-Camera Fusion
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

    # Add all nodes and launches to the launch description
    ld.add_action(rviz_node)
    ld.add_action(lidar_camera_fusion_node)
    ld.add_action(yolo_launch_fusion)

    return ld
