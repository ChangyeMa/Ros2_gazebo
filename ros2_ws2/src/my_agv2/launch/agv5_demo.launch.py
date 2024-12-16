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
    world_file_path = "world/test_1.world"
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
                                    '-x', '0.0',
                                    '-y', '-6.9',
                                    '-z', '0.1',
                                    '-R', '1.57',
                                    '-P', '0',
                                    '-Y', '0.0'],
                        output='screen')

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
    #     output='screen'
    # )

    # load_imu_sensor_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 'imu_sensor_broadcaster'],
    #     output='screen'
    # )

    # =========== add predefined tag detection node ===========
    tag_detection_node = Node(
        package='agv_control_pkg',
        executable='tag_detection',
        name='tag_detection',
        output='screen'
    )

    # =========== Load Rviz2 to visualize the camera ===========
    load_rviz = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', os.path.join(pkg_path, 'rviz', 'show_camera.rviz')],
                        output='screen')

    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_trajectory_controller,
        #         on_exit=[load_imu_sensor_broadcaster],
        #     )
        # ),

        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        # tag_detection_node,
        load_rviz,
    ])

    # ld.add_action(gazebo)
    # ld.add_action(node_robot_state_publisher)
    # ld.add_action(spawn_entity)
    # ld.add_action(tag_detection_node)
    # ld.add_action(load_rviz)

    return ld
