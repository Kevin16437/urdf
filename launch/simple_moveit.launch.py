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
    
    # Set the path to the URDF file
    urdf_file = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        'urdf/arm2.0.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Set the path to the SRDF file
    srdf_file = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        'config/arm2.srdf')
    
    # Read the SRDF file
    with open(srdf_file, 'r') as infp:
        robot_desc_semantic = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true')

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(FindPackageShare(pkg_name).find(pkg_name), 'rviz', 'urdf.rviz')],
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(static_tf_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)

    return ld
