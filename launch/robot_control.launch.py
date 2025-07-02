#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # Set the path to the controller config file
    controller_config_file = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        'config',
        'controllers.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use sim time if true')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Use Gazebo if true')

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-s', '-r', '--verbose', '-v', '4', 'empty.sdf'],
        output='screen')

    # Start Gazebo client    
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-g'],
        output='screen')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}])

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc},
                    controller_config_file,
                    {'use_sim_time': use_sim_time}],
        output='screen')

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        condition=IfCondition(use_gazebo),
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'arm2_robot'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Load joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load arm controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # Joint state publisher GUI (for manual control when not using Gazebo)
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(PythonExpression(['not ', use_gazebo])),
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
    ld.add_action(declare_use_gazebo_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_entity_node)
    
    # Add delayed controller loading
    ld.add_action(TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster]
    ))
    ld.add_action(TimerAction(
        period=5.0,
        actions=[load_arm_controller]
    ))
    
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
