#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
import numpy as np
import math
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from rclpy.action import ActionClient


class CartesianPathPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_path_planner')
        
        # Initialize MoveIt action client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            DisplayTrajectory, 
            'move_group/display_planned_path', 
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            'cartesian_path_markers', 
            10
        )
        
        # Services
        self.linear_path_srv = self.create_service(
            Trigger, 
            'plan_linear_path', 
            self.plan_linear_path_callback
        )
        self.circular_path_srv = self.create_service(
            Trigger, 
            'plan_circular_path', 
            self.plan_circular_path_callback
        )
        self.execute_path_srv = self.create_service(
            Trigger, 
            'execute_planned_path', 
            self.execute_path_callback
        )
        
        # Robot parameters
        self.planning_group = 'manipulator'
        self.end_effector_link = 'Link6'
        self.base_frame = 'base_link'
        
        # Path storage
        self.current_trajectory = None
        
        # Wait for MoveIt to be ready
        self.get_logger().info('Waiting for MoveIt action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveIt action server ready!')
        
        self.get_logger().info('Cartesian path planner ready!')
        self.get_logger().info('Available services:')
        self.get_logger().info('  - /plan_linear_path')
        self.get_logger().info('  - /plan_circular_path')
        self.get_logger().info('  - /execute_planned_path')

    def plan_linear_path_callback(self, request, response):
        """Plan a linear path"""
        self.get_logger().info('Planning linear path...')
        
        try:
            # Define start and end poses for linear path
            waypoints = self.generate_linear_waypoints()
            
            # Visualize waypoints
            self.visualize_waypoints(waypoints, 'linear_path')
            
            # Plan cartesian path
            success = self.plan_cartesian_path(waypoints)
            
            if success:
                response.success = True
                response.message = 'Linear path planned successfully'
                self.get_logger().info('Linear path planned successfully')
            else:
                response.success = False
                response.message = 'Failed to plan linear path'
                self.get_logger().error('Failed to plan linear path')
                
        except Exception as e:
            response.success = False
            response.message = f'Error planning linear path: {str(e)}'
            self.get_logger().error(f'Error planning linear path: {str(e)}')
        
        return response

    def plan_circular_path_callback(self, request, response):
        """Plan a circular path"""
        self.get_logger().info('Planning circular path...')
        
        try:
            # Define waypoints for circular path
            waypoints = self.generate_circular_waypoints()
            
            # Visualize waypoints
            self.visualize_waypoints(waypoints, 'circular_path')
            
            # Plan cartesian path
            success = self.plan_cartesian_path(waypoints)
            
            if success:
                response.success = True
                response.message = 'Circular path planned successfully'
                self.get_logger().info('Circular path planned successfully')
            else:
                response.success = False
                response.message = 'Failed to plan circular path'
                self.get_logger().error('Failed to plan circular path')
                
        except Exception as e:
            response.success = False
            response.message = f'Error planning circular path: {str(e)}'
            self.get_logger().error(f'Error planning circular path: {str(e)}')
        
        return response

    def execute_path_callback(self, request, response):
        """Execute the planned path"""
        self.get_logger().info('Executing planned path...')
        
        if self.current_trajectory is None:
            response.success = False
            response.message = 'No trajectory to execute. Plan a path first.'
            self.get_logger().error('No trajectory to execute')
            return response
        
        try:
            # Execute the trajectory
            success = self.execute_trajectory(self.current_trajectory)
            
            if success:
                response.success = True
                response.message = 'Path executed successfully'
                self.get_logger().info('Path executed successfully')
            else:
                response.success = False
                response.message = 'Failed to execute path'
                self.get_logger().error('Failed to execute path')
                
        except Exception as e:
            response.success = False
            response.message = f'Error executing path: {str(e)}'
            self.get_logger().error(f'Error executing path: {str(e)}')
        
        return response

    def generate_linear_waypoints(self):
        """Generate waypoints for a linear path"""
        waypoints = []
        
        # Start pose
        start_pose = Pose()
        start_pose.position.x = 0.3
        start_pose.position.y = 0.0
        start_pose.position.z = 0.4
        start_pose.orientation.w = 1.0
        waypoints.append(start_pose)
        
        # End pose (move 0.2m in X direction)
        end_pose = Pose()
        end_pose.position.x = 0.5
        end_pose.position.y = 0.0
        end_pose.position.z = 0.4
        end_pose.orientation.w = 1.0
        waypoints.append(end_pose)
        
        # Interpolate intermediate waypoints
        interpolated_waypoints = []
        num_points = 10
        
        for i in range(num_points + 1):
            t = i / num_points
            pose = Pose()
            pose.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
            pose.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
            pose.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)
            pose.orientation.w = 1.0
            interpolated_waypoints.append(pose)
        
        return interpolated_waypoints

    def generate_circular_waypoints(self):
        """Generate waypoints for a circular path"""
        waypoints = []
        
        # Circle parameters
        center_x = 0.4
        center_y = 0.0
        center_z = 0.4
        radius = 0.1
        num_points = 20
        
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            
            pose = Pose()
            pose.position.x = center_x + radius * math.cos(angle)
            pose.position.y = center_y + radius * math.sin(angle)
            pose.position.z = center_z
            pose.orientation.w = 1.0
            
            waypoints.append(pose)
        
        return waypoints

    def plan_cartesian_path(self, waypoints):
        """Plan cartesian path using MoveIt (simplified version)"""
        # This is a simplified implementation
        # In a real implementation, you would use MoveIt's compute_cartesian_path service
        
        self.get_logger().info(f'Planning cartesian path with {len(waypoints)} waypoints')
        
        # For now, we'll just store the waypoints as a simple trajectory
        # In practice, you'd call MoveIt's cartesian path planning service
        self.current_trajectory = waypoints
        
        return True

    def execute_trajectory(self, trajectory):
        """Execute trajectory using MoveIt"""
        # This is a simplified implementation
        # In practice, you'd send the trajectory to MoveIt for execution
        
        self.get_logger().info('Executing trajectory...')
        
        # Simulate execution
        import time
        time.sleep(2)
        
        return True

    def visualize_waypoints(self, waypoints, path_type):
        """Visualize waypoints as markers in RViz"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Create markers for waypoints
        for i, pose in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = path_type
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = pose
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            
            if path_type == 'linear_path':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # circular_path
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        # Create line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = self.base_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = path_type + '_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        for pose in waypoints:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            line_marker.points.append(point)
        
        line_marker.scale.x = 0.005
        
        if path_type == 'linear_path':
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
        else:  # circular_path
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
        
        line_marker.color.a = 0.8
        marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianPathPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
