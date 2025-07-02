#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
import numpy as np
import struct
import math
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import threading


class WorkspaceAnalyzer(Node):
    def __init__(self):
        super().__init__('workspace_analyzer')
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            'reachable_workspace', 
            10
        )
        
        # Services
        self.analyze_workspace_srv = self.create_service(
            Trigger, 
            'analyze_workspace', 
            self.analyze_workspace_callback
        )
        
        # Service clients
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        
        # Robot parameters
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.joint_limits = {
            'j1': (-3.14159, 3.14159),
            'j2': (-1.57, 1.57),
            'j3': (-1.57, 1.57),
            'j4': (-3.14159, 3.14159),
            'j5': (-1.57, 1.57),
            'j6': (-3.14159, 3.14159)
        }
        self.end_effector_link = 'Link6'
        
        # Analysis parameters
        self.resolution = 0.2  # Step size in radians
        self.max_points = 50000  # Maximum points to compute
        
        # Storage for reachable points
        self.reachable_points = []
        
        # Wait for FK service
        self.get_logger().info('Waiting for forward kinematics service...')
        self.fk_client.wait_for_service()
        self.get_logger().info('Forward kinematics service ready!')
        
        self.get_logger().info('Workspace analyzer ready!')
        self.get_logger().info('Available services:')
        self.get_logger().info('  - /analyze_workspace')

    def analyze_workspace_callback(self, request, response):
        """Analyze the robot's reachable workspace"""
        self.get_logger().info('Starting workspace analysis...')
        
        try:
            # Clear previous results
            self.reachable_points = []
            
            # Start analysis in a separate thread to avoid blocking
            analysis_thread = threading.Thread(target=self.compute_workspace)
            analysis_thread.start()
            
            response.success = True
            response.message = 'Workspace analysis started. Check /reachable_workspace topic for results.'
            self.get_logger().info('Workspace analysis started in background')
            
        except Exception as e:
            response.success = False
            response.message = f'Error starting workspace analysis: {str(e)}'
            self.get_logger().error(f'Error starting workspace analysis: {str(e)}')
        
        return response

    def compute_workspace(self):
        """Compute the reachable workspace by sampling joint space"""
        self.get_logger().info('Computing reachable workspace...')
        
        total_combinations = 1
        for joint_name in self.joint_names:
            lower, upper = self.joint_limits[joint_name]
            steps = int((upper - lower) / self.resolution) + 1
            total_combinations *= steps
        
        self.get_logger().info(f'Total joint combinations to check: {total_combinations}')
        self.get_logger().info(f'Limiting to {self.max_points} points for performance')
        
        points_computed = 0
        points_added = 0
        
        # Sample joint space
        for j1 in self.sample_joint_range('j1'):
            for j2 in self.sample_joint_range('j2'):
                for j3 in self.sample_joint_range('j3'):
                    for j4 in self.sample_joint_range('j4'):
                        for j5 in self.sample_joint_range('j5'):
                            for j6 in self.sample_joint_range('j6'):
                                
                                if points_computed >= self.max_points:
                                    break
                                
                                joint_values = [j1, j2, j3, j4, j5, j6]
                                
                                # Compute forward kinematics
                                end_effector_pose = self.compute_fk(joint_values)
                                
                                if end_effector_pose is not None:
                                    point = [
                                        end_effector_pose.position.x,
                                        end_effector_pose.position.y,
                                        end_effector_pose.position.z
                                    ]
                                    self.reachable_points.append(point)
                                    points_added += 1
                                
                                points_computed += 1
                                
                                # Publish intermediate results every 1000 points
                                if points_computed % 1000 == 0:
                                    self.get_logger().info(f'Computed {points_computed} points, added {points_added} valid points')
                                    self.publish_pointcloud()
                            
                            if points_computed >= self.max_points:
                                break
                        if points_computed >= self.max_points:
                            break
                    if points_computed >= self.max_points:
                        break
                if points_computed >= self.max_points:
                    break
            if points_computed >= self.max_points:
                break
        
        self.get_logger().info(f'Workspace analysis complete! Found {points_added} reachable points')
        
        # Publish final point cloud
        self.publish_pointcloud()

    def sample_joint_range(self, joint_name):
        """Generate sample values for a joint within its limits"""
        lower, upper = self.joint_limits[joint_name]
        
        # Use coarser resolution for joints with larger ranges
        if joint_name in ['j1', 'j4', 'j6']:
            resolution = self.resolution * 2  # Coarser for continuous rotation joints
        else:
            resolution = self.resolution
        
        current = lower
        while current <= upper:
            yield current
            current += resolution

    def compute_fk(self, joint_values):
        """Compute forward kinematics for given joint values"""
        try:
            # Create FK request
            request = GetPositionFK.Request()
            
            # Set robot state
            robot_state = RobotState()
            joint_state = JointState()
            joint_state.name = self.joint_names
            joint_state.position = joint_values
            robot_state.joint_state = joint_state
            request.robot_state = robot_state
            
            # Set links to compute FK for
            request.fk_link_names = [self.end_effector_link]
            
            # Call service
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == response.error_code.SUCCESS:
                    if len(response.pose_stamped) > 0:
                        return response.pose_stamped[0].pose
            
            return None
            
        except Exception as e:
            self.get_logger().debug(f'FK computation failed: {str(e)}')
            return None

    def publish_pointcloud(self):
        """Publish the reachable points as a point cloud"""
        if not self.reachable_points:
            return
        
        # Create PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = 'base_link'
        
        # Define point fields (x, y, z)
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set point cloud properties
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12  # 3 floats * 4 bytes each
        pointcloud.row_step = pointcloud.point_step * len(self.reachable_points)
        pointcloud.is_dense = True
        pointcloud.width = len(self.reachable_points)
        pointcloud.height = 1
        
        # Pack point data
        point_data = []
        for point in self.reachable_points:
            point_data.extend(struct.pack('fff', point[0], point[1], point[2]))
        
        pointcloud.data = point_data
        
        # Publish point cloud
        self.pointcloud_pub.publish(pointcloud)
        
        self.get_logger().info(f'Published point cloud with {len(self.reachable_points)} points')

    def create_sample_workspace(self):
        """Create a sample workspace for testing (sphere-like shape)"""
        self.get_logger().info('Creating sample workspace for testing...')
        
        # Generate points in a sphere-like pattern
        center = [0.4, 0.0, 0.4]
        radius = 0.3
        
        for i in range(1000):
            # Random spherical coordinates
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            r = np.random.uniform(0.1, radius)
            
            # Convert to Cartesian
            x = center[0] + r * np.sin(phi) * np.cos(theta)
            y = center[1] + r * np.sin(phi) * np.sin(theta)
            z = center[2] + r * np.cos(phi)
            
            # Add some constraints to make it more realistic
            if z > 0.1:  # Above ground
                self.reachable_points.append([x, y, z])
        
        self.publish_pointcloud()


def main(args=None):
    rclpy.init(args=args)
    
    node = WorkspaceAnalyzer()
    
    # Create sample workspace for testing
    node.create_sample_workspace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
