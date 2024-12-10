import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(
        get_package_share_directory('agv_controller'),)

    ld = LaunchDescription()

    # Start the aptag_detection node
    aptag_detection_node = Node(
        package='aptag_detection',
        executable='tag_detection_node',
        name='aptag_detection_node',
        output='screen'
    )

    # Start the aptag_detection node
    aptag_detection_node2 = Node(
        package='aptag_detection',
        executable='tag_detection_node',
        name='aptag_detection_node_rear',
        output='screen',
        parameters=[
            {"camera_number": 2},
            {"camera_index": 2},
        ]
    )

    # start agv_controller_node
    agv_controller_node = Node(
        package='agv_controller',
        executable='agv_controller',
        name='agv_controller',
        output='screen'
    )

    # start tf2_broadcaster_node
    tf_broadcaster_node = Node(
        package='aptag_detection',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        output='screen'
    )

    # start static_tf_broadcaster
    static_tf_broadcaster_node = Node(
        package='aptag_detection',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen'
    )

    # start pose to marker node
    pose_to_marker_node = Node(
        package='aptag_detection',
        executable='pose_to_marker_node',
        name='pose_to_marker_node',
        output='screen'
    )


    # load rviz2 with the configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz_config', 'AMR_control.rviz')], 
    )

    # Add the actions to the launch description

    ld.add_action(aptag_detection_node)
    ld.add_action(aptag_detection_node2)
    ld.add_action(static_tf_broadcaster_node)
    # ld.add_action(agv_controller_node)
    ld.add_action(pose_to_marker_node)
    ld.add_action(tf_broadcaster_node)
    
    # ld.add_action(rviz_node)

    return ld