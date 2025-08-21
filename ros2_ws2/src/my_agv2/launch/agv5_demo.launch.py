# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(
        get_package_share_directory('my_agv2'))

    # =========== add predefined world file ===========
    # world_file_path = "world/test_1.world"
    world_file_path = "world/catalyst_env_v3.world"
    world_path = os.path.join(pkg_path, world_file_path)

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    # =========== add predefined robot description ===========
    xacro_file = os.path.join(pkg_path,
                              'urdf',
                              'AGV5.xacro.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # spawn the agv at the specified coordinates and orientation
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cart',
                                    '-x', '12.0',
                                    '-y', '-7.0',
                                    '-z', '0.1',
                                    '-R', '1.57',
                                    '-P', '0.0',
                                    '-Y', '0.0'],
                        output='screen')

    # =========== Load Rviz2 to visualize the camera ===========
    load_rviz = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', os.path.join(pkg_path, 'rviz', 'show_camera.rviz')],
                        output='screen')

    # add tf static transform publisher from lidar to base_link
    lidar_static_tf_node=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        output="screen",
        arguments=["0.0", "0.6","0", "0","-1.57","1.57","base_link","lidar_link"],
    )

    # add tf static transform publisher from imu_link to base_link
    imu_static_tf_node=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_tf",
        output="screen",
        arguments=["0.0", "0.0","0.0", "0.0","0.0","0.0","base_link","imu_link"],
    )
    
    # add tf static transform publisher from base_link to camera_1_link
    camera_1_static_tf_node=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_1_static_tf",
        output="screen",
        arguments=["0.44", "0.2","0", "-1.57","-1.57","-1.57","base_link","camera_1_link"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        lidar_static_tf_node,
        imu_static_tf_node,
        camera_1_static_tf_node,
        load_rviz,
    ])