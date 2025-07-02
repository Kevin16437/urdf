#!/usr/bin/env python3
# Make script executable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import numpy as np


class InteractiveTeachPendant(Node):
    def __init__(self):
        super().__init__('interactive_teach_pendant')
        
        # Initialize interactive marker server
        self.server = InteractiveMarkerServer(self, 'teach_pendant')
        
        # Initialize MoveIt action client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot parameters
        self.planning_group = 'manipulator'
        self.end_effector_link = 'Link6'
        self.base_frame = 'base_link'
        
        # Wait for MoveIt to be ready
        self.get_logger().info('Waiting for MoveIt action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveIt action server ready!')
        
        # Create interactive marker
        self.create_interactive_marker()
        
        # Apply changes
        self.server.applyChanges()
        
        self.get_logger().info('Interactive teach pendant ready!')

    def create_interactive_marker(self):
        """Create an interactive marker at the end effector position"""
        
        # Get current end effector pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.end_effector_link, 
                rclpy.time.Time()
            )
            
            # Create interactive marker
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = self.base_frame
            int_marker.header.stamp = self.get_clock().now().to_msg()
            int_marker.name = "end_effector_control"
            int_marker.description = "End Effector Control"
            
            # Set initial position from transform
            int_marker.pose.position.x = transform.transform.translation.x
            int_marker.pose.position.y = transform.transform.translation.y
            int_marker.pose.position.z = transform.transform.translation.z
            int_marker.pose.orientation = transform.transform.rotation
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            # Use default position if transform lookup fails
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = self.base_frame
            int_marker.header.stamp = self.get_clock().now().to_msg()
            int_marker.name = "end_effector_control"
            int_marker.description = "End Effector Control"
            int_marker.pose.position.x = 0.5
            int_marker.pose.position.y = 0.0
            int_marker.pose.position.z = 0.5
            int_marker.pose.orientation.w = 1.0

        # Create a sphere marker for visualization
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8

        # Create control for the sphere
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)

        # Create 6DOF controls
        # Translation controls
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        # Rotation controls
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)

        # Add the interactive marker to the server
        self.server.insert(int_marker, self.process_feedback)

    def process_feedback(self, feedback):
        """Process feedback from interactive marker"""
        self.get_logger().info(f'Marker moved to: x={feedback.pose.position.x:.3f}, '
                              f'y={feedback.pose.position.y:.3f}, '
                              f'z={feedback.pose.position.z:.3f}')
        
        # Only plan and execute on mouse button release
        if feedback.event_type == feedback.MOUSE_UP:
            self.plan_and_execute_to_pose(feedback.pose)

    def plan_and_execute_to_pose(self, target_pose):
        """Plan and execute motion to target pose using MoveIt"""
        self.get_logger().info('Planning motion to target pose...')
        
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        
        # Set planning group
        goal.request.group_name = self.planning_group
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        # Set position and orientation constraints
        goal.request.goal_constraints.append(self.create_pose_goal(pose_stamped))
        
        # Set planning parameters
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def create_pose_goal(self, pose_stamped):
        """Create pose goal constraints"""
        constraints = Constraints()
        
        # This is a simplified version - in practice you'd want to use
        # proper pose constraints from moveit_msgs
        # For now, we'll return empty constraints
        return constraints

    def goal_response_callback(self, future):
        """Handle MoveIt goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt')
            return
        
        self.get_logger().info('Goal accepted by MoveIt')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle MoveIt execution result"""
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Motion executed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')


def main(args=None):
    rclpy.init(args=args)
    
    node = InteractiveTeachPendant()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
