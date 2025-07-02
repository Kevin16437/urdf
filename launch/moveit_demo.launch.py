#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


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

    # Set the path to kinematics config
    kinematics_config = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        'config/kinematics.yaml')

    # Set the path to joint limits config
    joint_limits_config = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        'config/joint_limits.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'rviz',
            'moveit.rviz'
        ]),
        description='RViz configuration file')

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"robot_description_semantic": robot_desc_semantic},
            kinematics_config,
            joint_limits_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            {"robot_description": robot_desc},
            {"robot_description_semantic": robot_desc_semantic},
            kinematics_config,
            joint_limits_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        "config",
        "moveit_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_desc}, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_cmd)

    # Add any actions
    ld.add_action(static_tf_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld
