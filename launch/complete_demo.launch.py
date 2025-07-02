#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to different files and folders.
    pkg_name = 'arm2_description'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_interactive_teach = LaunchConfiguration('enable_interactive_teach')
    enable_cartesian_planning = LaunchConfiguration('enable_cartesian_planning')
    enable_workspace_analysis = LaunchConfiguration('enable_workspace_analysis')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true')

    declare_interactive_teach_cmd = DeclareLaunchArgument(
        name='enable_interactive_teach',
        default_value='true',
        description='Enable interactive teach pendant')

    declare_cartesian_planning_cmd = DeclareLaunchArgument(
        name='enable_cartesian_planning',
        default_value='true',
        description='Enable cartesian path planning')

    declare_workspace_analysis_cmd = DeclareLaunchArgument(
        name='enable_workspace_analysis',
        default_value='true',
        description='Enable workspace analysis')

    # Include MoveIt demo launch
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'moveit_demo.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Interactive teach pendant node
    interactive_teach_node = Node(
        package=pkg_name,
        executable='interactive_teach_pendant.py',
        name='interactive_teach_pendant',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfiguration('enable_interactive_teach')
    )

    # Cartesian path planner node
    cartesian_planner_node = Node(
        package=pkg_name,
        executable='cartesian_path_planner.py',
        name='cartesian_path_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfiguration('enable_cartesian_planning')
    )

    # Workspace analyzer node
    workspace_analyzer_node = Node(
        package=pkg_name,
        executable='workspace_analyzer.py',
        name='workspace_analyzer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfiguration('enable_workspace_analysis')
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_interactive_teach_cmd)
    ld.add_action(declare_cartesian_planning_cmd)
    ld.add_action(declare_workspace_analysis_cmd)

    # Add any actions
    ld.add_action(moveit_demo_launch)
    ld.add_action(interactive_teach_node)
    ld.add_action(cartesian_planner_node)
    ld.add_action(workspace_analyzer_node)

    return ld
