#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time


class UsageGuide(Node):
    def __init__(self):
        super().__init__('usage_guide')
        
        self.print_welcome()
        self.print_usage_instructions()
        
    def print_welcome(self):
        """Print welcome message"""
        print("\n" + "="*60)
        print("🤖 ARM2 Robot Simulation and Analysis Package")
        print("="*60)
        print("Welcome to the comprehensive ROS 2 robotic arm simulation!")
        print("This package provides advanced features for robot control,")
        print("motion planning, and workspace analysis.")
        print("="*60 + "\n")
    
    def print_usage_instructions(self):
        """Print detailed usage instructions"""
        
        print("📋 AVAILABLE LAUNCH COMMANDS:")
        print("-" * 40)
        
        commands = [
            {
                "name": "1. Basic Visualization with Joint Control",
                "command": "ros2 launch arm2_description display_with_gui.launch.py",
                "description": "Opens RViz with robot model and joint control sliders"
            },
            {
                "name": "2. MoveIt Motion Planning",
                "command": "ros2 launch arm2_description moveit_demo.launch.py",
                "description": "Full MoveIt environment with collision detection"
            },
            {
                "name": "3. Interactive Teaching",
                "command": "ros2 launch arm2_description interactive_teach.launch.py",
                "description": "Drag-and-drop end-effector control in RViz"
            },
            {
                "name": "4. Cartesian Path Planning",
                "command": "ros2 launch arm2_description cartesian_planning.launch.py",
                "description": "Linear and circular trajectory planning"
            },
            {
                "name": "5. Workspace Analysis",
                "command": "ros2 launch arm2_description workspace_analysis.launch.py",
                "description": "3D visualization of reachable workspace"
            },
            {
                "name": "6. Complete Demo (All Features)",
                "command": "ros2 launch arm2_description complete_demo.launch.py",
                "description": "All features combined in one launch"
            }
        ]
        
        for cmd in commands:
            print(f"\n{cmd['name']}:")
            print(f"  Command: {cmd['command']}")
            print(f"  Description: {cmd['description']}")
        
        print("\n" + "="*60)
        print("🔧 SERVICE COMMANDS:")
        print("-" * 40)
        
        services = [
            {
                "name": "Plan Linear Path",
                "command": "ros2 service call /plan_linear_path std_srvs/srv/Trigger",
                "description": "Plans a straight-line trajectory"
            },
            {
                "name": "Plan Circular Path", 
                "command": "ros2 service call /plan_circular_path std_srvs/srv/Trigger",
                "description": "Plans a circular trajectory"
            },
            {
                "name": "Execute Planned Path",
                "command": "ros2 service call /execute_planned_path std_srvs/srv/Trigger",
                "description": "Executes the previously planned path"
            },
            {
                "name": "Analyze Workspace",
                "command": "ros2 service call /analyze_workspace std_srvs/srv/Trigger",
                "description": "Computes and visualizes reachable workspace"
            }
        ]
        
        for svc in services:
            print(f"\n{svc['name']}:")
            print(f"  Command: {svc['command']}")
            print(f"  Description: {svc['description']}")
        
        print("\n" + "="*60)
        print("📊 MONITORING COMMANDS:")
        print("-" * 40)
        
        monitoring = [
            {
                "name": "View Joint States",
                "command": "ros2 topic echo /joint_states"
            },
            {
                "name": "View Robot Description",
                "command": "ros2 topic echo /robot_description"
            },
            {
                "name": "View Point Cloud",
                "command": "ros2 topic echo /reachable_workspace"
            },
            {
                "name": "List All Topics",
                "command": "ros2 topic list"
            },
            {
                "name": "List All Services",
                "command": "ros2 service list"
            }
        ]
        
        for mon in monitoring:
            print(f"  {mon['name']}: {mon['command']}")
        
        print("\n" + "="*60)
        print("🚀 QUICK START GUIDE:")
        print("-" * 40)
        print("1. First time setup:")
        print("   cd ~/urdf_ws")
        print("   source /opt/ros/jazzy/setup.bash")
        print("   colcon build")
        print("   source install/setup.bash")
        print()
        print("2. Start with basic visualization:")
        print("   ros2 launch arm2_description display_with_gui.launch.py")
        print()
        print("3. Try interactive teaching:")
        print("   ros2 launch arm2_description interactive_teach.launch.py")
        print()
        print("4. Explore workspace analysis:")
        print("   ros2 launch arm2_description workspace_analysis.launch.py")
        print("   ros2 service call /analyze_workspace std_srvs/srv/Trigger")
        
        print("\n" + "="*60)
        print("⚠️  TROUBLESHOOTING:")
        print("-" * 40)
        print("• If MoveIt fails to plan: Check joint limits and increase planning time")
        print("• If interactive markers don't appear: Restart RViz")
        print("• If point cloud is empty: Wait for workspace analysis to complete")
        print("• If services are unavailable: Ensure all nodes are running")
        
        print("\n" + "="*60)
        print("📚 For more information, see README.md")
        print("🐛 Report issues with detailed error logs")
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    guide = UsageGuide()
    
    try:
        # Keep the node alive briefly
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        guide.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
