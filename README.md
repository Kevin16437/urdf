# ARM2 Robot Simulation and Analysis Package

This package provides a comprehensive ROS 2 simulation and analysis environment for the ARM2 6-DOF robotic manipulator. It includes advanced features like MoveIt integration, interactive teaching, Cartesian path planning, and workspace analysis.

## Features

- **URDF Model**: Complete robot description with accurate kinematics and dynamics
- **Gazebo Simulation**: Full physics simulation with ros2_control integration
- **MoveIt Integration**: Motion planning with collision avoidance
- **Interactive Teaching**: Drag-and-drop end-effector control in RViz
- **Cartesian Path Planning**: Linear and circular trajectory generation
- **Workspace Analysis**: 3D visualization of reachable space as point clouds
- **GUI Control**: Joint-level control with sliders

## Prerequisites

- Ubuntu 24.04 with WSL2
- ROS 2 Jazzy Jalisco
- Required packages:
  ```bash
  sudo apt install ros-jazzy-moveit ros-jazzy-gazebo-ros-pkgs ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-publisher-gui
  ```

## Installation

1. Create workspace and clone:
   ```bash
   mkdir -p ~/urdf_ws/src
   cd ~/urdf_ws/src
   # Copy this package to src directory
   ```

2. Build the package:
   ```bash
   cd ~/urdf_ws
   source /opt/ros/jazzy/setup.bash
   colcon build
   source install/setup.bash
   ```

## Usage

### 1. Basic Visualization with Joint Control

Launch RViz with joint state publisher GUI:
```bash
ros2 launch arm2_description display_with_gui.launch.py
```

This opens RViz with the robot model and a GUI to control individual joints.

### 2. MoveIt Motion Planning

Launch the complete MoveIt environment:
```bash
ros2 launch arm2_description moveit_demo.launch.py
```

Features:
- Interactive motion planning in RViz
- Collision detection and avoidance
- Pre-defined poses (home, ready)
- Path visualization

### 3. Interactive Teaching

Launch interactive teaching mode:
```bash
ros2 launch arm2_description interactive_teach.launch.py
```

Features:
- Drag the red sphere in RViz to move the end-effector
- Automatic motion planning to target poses
- Real-time visualization

### 4. Cartesian Path Planning

Launch Cartesian path planning:
```bash
ros2 launch arm2_description cartesian_planning.launch.py
```

Available services:
```bash
# Plan linear path
ros2 service call /plan_linear_path std_srvs/srv/Trigger

# Plan circular path
ros2 service call /plan_circular_path std_srvs/srv/Trigger

# Execute planned path
ros2 service call /execute_planned_path std_srvs/srv/Trigger
```

### 5. Workspace Analysis

Launch workspace analysis:
```bash
ros2 launch arm2_description workspace_analysis.launch.py
```

Analyze reachable workspace:
```bash
ros2 service call /analyze_workspace std_srvs/srv/Trigger
```

The reachable points will be published as a point cloud on `/reachable_workspace` topic.

### 6. Complete Demo

Launch all features together:
```bash
ros2 launch arm2_description complete_demo.launch.py
```

Optional parameters:
```bash
ros2 launch arm2_description complete_demo.launch.py enable_interactive_teach:=true enable_cartesian_planning:=true enable_workspace_analysis:=true
```

## Package Structure

```
arm2_description/
├── config/                 # Configuration files
│   ├── controllers.yaml    # ros2_control configuration
│   ├── moveit_controllers.yaml
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   └── arm2.srdf          # Semantic robot description
├── launch/                # Launch files
│   ├── display_with_gui.launch.py
│   ├── moveit_demo.launch.py
│   ├── interactive_teach.launch.py
│   ├── cartesian_planning.launch.py
│   ├── workspace_analysis.launch.py
│   └── complete_demo.launch.py
├── meshes/                # 3D mesh files
├── rviz/                  # RViz configurations
│   ├── urdf.rviz
│   └── moveit.rviz
├── scripts/               # Python nodes
│   ├── interactive_teach_pendant.py
│   ├── cartesian_path_planner.py
│   └── workspace_analyzer.py
├── urdf/                  # Robot description
│   └── arm2.0.urdf
└── worlds/                # Gazebo world files
    └── empty.world
```

## Troubleshooting

### Common Issues

1. **MoveIt planning fails**:
   - Check joint limits in URDF
   - Verify SRDF collision matrix
   - Increase planning time

2. **Interactive markers not visible**:
   - Ensure interactive_markers package is installed
   - Check TF tree for missing transforms

3. **Point cloud not showing**:
   - Add PointCloud2 display in RViz
   - Set topic to `/reachable_workspace`
   - Check frame_id matches

### Performance Tips

- Reduce workspace analysis resolution for faster computation
- Use coarser joint sampling for large workspaces
- Limit maximum points in workspace analysis

## Development

### Adding New Features

1. Create new Python scripts in `scripts/` directory
2. Add corresponding launch files in `launch/`
3. Update CMakeLists.txt to install new files
4. Test with `colcon build`

### Customizing Robot Parameters

- Modify joint limits in `urdf/arm2.0.urdf`
- Update controller parameters in `config/controllers.yaml`
- Adjust MoveIt settings in `config/` files

## License

BSD 3-Clause License

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Support

For issues and questions:
- Check the troubleshooting section
- Review ROS 2 and MoveIt documentation
- Open an issue with detailed error logs
