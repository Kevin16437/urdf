#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true')

    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}])

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}])

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(FindPackageShare(pkg_name).find(pkg_name), 'rviz', 'urdf.rviz')],
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)

    # Add any actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
