import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
'''
def generate_launch_description():

    ld = LaunchDescription()
    
    # urdf = os.path.join(get_package_share_directory("five_bar_link"), "urdf/FiveBarLink.urdf")
    urdf = os.path.join(get_package_share_directory("my_agv"), "urdf/AGV3_single.urdf")

    if urdf != "":
            print(f"Found URDF PATH {urdf}")

    with open(urdf, 'r') as uf:
        robot_desc = uf.read()

    gazebo_world = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('gazebo_ros'),
                            'launch', 'gazebo.launch.py')))

    start_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot_model', '-file', urdf,
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.1',
                                    '-R', '1.57',
                                    '-P', '0',
                                    '-Y', '0'],
                        output='screen')
    

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_agv"),
            "config",
            "controller_configuration.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf],
        output='screen')

    start_joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_joint_state_gui)
    ld.add_action(gazebo_world)
    ld.add_action(start_spawn_entity)
    ld.add_action(control_node)

    return ld

if __name__ == "__main__":
    generate_launch_description()
'''
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('my_agv'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'AGV3_single.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cart',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.1',
                                    '-R', '1.57',
                                    '-P', '0',
                                    '-Y', '0'],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])