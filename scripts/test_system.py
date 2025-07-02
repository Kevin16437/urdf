#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import subprocess
import sys


class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        self.get_logger().info('ARM2 Robot System Tester Started')
        
        # Test results
        self.test_results = {}
        
    def test_basic_functionality(self):
        """Test basic ROS 2 functionality"""
        self.get_logger().info('Testing basic ROS 2 functionality...')
        
        try:
            # Test if we can create publishers and subscribers
            test_pub = self.create_publisher(std_msgs.msg.String, 'test_topic', 10)
            test_sub = self.create_subscription(std_msgs.msg.String, 'test_topic', lambda msg: None, 10)
            
            self.test_results['basic_ros2'] = True
            self.get_logger().info('✓ Basic ROS 2 functionality: PASSED')
            
        except Exception as e:
            self.test_results['basic_ros2'] = False
            self.get_logger().error(f'✗ Basic ROS 2 functionality: FAILED - {str(e)}')
    
    def test_services_available(self):
        """Test if required services are available"""
        self.get_logger().info('Testing service availability...')
        
        required_services = [
            '/plan_linear_path',
            '/plan_circular_path', 
            '/execute_planned_path',
            '/analyze_workspace'
        ]
        
        available_services = []
        
        for service_name in required_services:
            try:
                # Check if service exists
                service_names = self.get_service_names_and_types()
                service_exists = any(service_name in name for name, _ in service_names)
                
                if service_exists:
                    available_services.append(service_name)
                    self.get_logger().info(f'✓ Service {service_name}: AVAILABLE')
                else:
                    self.get_logger().warn(f'✗ Service {service_name}: NOT AVAILABLE')
                    
            except Exception as e:
                self.get_logger().error(f'✗ Error checking service {service_name}: {str(e)}')
        
        self.test_results['services'] = len(available_services) == len(required_services)
        
        if self.test_results['services']:
            self.get_logger().info('✓ All required services: AVAILABLE')
        else:
            self.get_logger().warn(f'✗ Only {len(available_services)}/{len(required_services)} services available')
    
    def test_topics_available(self):
        """Test if required topics are available"""
        self.get_logger().info('Testing topic availability...')
        
        required_topics = [
            '/robot_description',
            '/joint_states',
            '/tf',
            '/tf_static'
        ]
        
        available_topics = []
        
        try:
            topic_names = self.get_topic_names_and_types()
            
            for topic_name in required_topics:
                topic_exists = any(topic_name in name for name, _ in topic_names)
                
                if topic_exists:
                    available_topics.append(topic_name)
                    self.get_logger().info(f'✓ Topic {topic_name}: AVAILABLE')
                else:
                    self.get_logger().warn(f'✗ Topic {topic_name}: NOT AVAILABLE')
                    
        except Exception as e:
            self.get_logger().error(f'✗ Error checking topics: {str(e)}')
        
        self.test_results['topics'] = len(available_topics) == len(required_topics)
        
        if self.test_results['topics']:
            self.get_logger().info('✓ All required topics: AVAILABLE')
        else:
            self.get_logger().warn(f'✗ Only {len(available_topics)}/{len(required_topics)} topics available')
    
    def test_cartesian_planning_service(self):
        """Test cartesian planning services"""
        self.get_logger().info('Testing cartesian planning services...')
        
        try:
            # Create service client
            linear_client = self.create_client(Trigger, '/plan_linear_path')
            
            if linear_client.wait_for_service(timeout_sec=5.0):
                # Call service
                request = Trigger.Request()
                future = linear_client.call_async(request)
                
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info('✓ Linear path planning service: WORKING')
                        self.test_results['cartesian_planning'] = True
                    else:
                        self.get_logger().warn(f'✗ Linear path planning failed: {response.message}')
                        self.test_results['cartesian_planning'] = False
                else:
                    self.get_logger().error('✗ Linear path planning service call failed')
                    self.test_results['cartesian_planning'] = False
            else:
                self.get_logger().error('✗ Linear path planning service not available')
                self.test_results['cartesian_planning'] = False
                
        except Exception as e:
            self.get_logger().error(f'✗ Error testing cartesian planning: {str(e)}')
            self.test_results['cartesian_planning'] = False
    
    def run_all_tests(self):
        """Run all system tests"""
        self.get_logger().info('='*50)
        self.get_logger().info('Starting ARM2 Robot System Tests')
        self.get_logger().info('='*50)
        
        # Run tests
        self.test_basic_functionality()
        time.sleep(1)
        
        self.test_topics_available()
        time.sleep(1)
        
        self.test_services_available()
        time.sleep(1)
        
        # Print summary
        self.print_test_summary()
    
    def print_test_summary(self):
        """Print test summary"""
        self.get_logger().info('='*50)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('='*50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)
        
        for test_name, result in self.test_results.items():
            status = "PASSED" if result else "FAILED"
            symbol = "✓" if result else "✗"
            self.get_logger().info(f'{symbol} {test_name}: {status}')
        
        self.get_logger().info('-'*50)
        self.get_logger().info(f'Total Tests: {total_tests}')
        self.get_logger().info(f'Passed: {passed_tests}')
        self.get_logger().info(f'Failed: {total_tests - passed_tests}')
        
        if passed_tests == total_tests:
            self.get_logger().info('🎉 ALL TESTS PASSED!')
        else:
            self.get_logger().warn(f'⚠️  {total_tests - passed_tests} TESTS FAILED')
        
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    tester = SystemTester()
    
    try:
        # Run tests
        tester.run_all_tests()
        
        # Keep node alive for a bit
        time.sleep(2)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
