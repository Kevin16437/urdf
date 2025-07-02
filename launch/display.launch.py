#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to different files and folders.
    pkg_name = 'arm2_description'
    file_subpath = 'urdf/arm2.0.urdf'
    
    # Set the path to the URDF file
    urdf_file = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        file_subpath)
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true')

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='true',
        description='Use ros2_control if true')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}])

    # Publish the joint states of the robot
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_ros2_control))

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(FindPackageShare(pkg_name).find(pkg_name), 'rviz', 'urdf.rviz')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('rviz'), "' != 'false'"
        ])),
        parameters=[{'use_sim_time': use_sim_time}])

    # Declare the launch arguments for rviz
    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Launch RViz if true')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_rviz_cmd)

    # Add any actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
