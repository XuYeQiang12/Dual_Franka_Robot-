#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class TestController(Node):
    def __init__(self):
        super().__init__('test_controller')

        
        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/joint_commands', 10)
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/joint_commands', 10)

        
        time.sleep(1.0)

        self.get_logger().info('Test Controller initialized')

    def send_home_position(self):
        """Send both arms to home position"""
        self.get_logger().info('Moving to home position...')

        home_pos = [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]

        left_msg = Float64MultiArray()
        left_msg.data = home_pos

        right_msg = Float64MultiArray()
        right_msg.data = home_pos

        self.left_cmd_pub.publish(left_msg)
        self.right_cmd_pub.publish(right_msg)

    def send_wave_motion(self, duration=5.0, frequency=0.5):
        """Send a waving motion to both arms"""
        self.get_logger().info(f'Performing wave motion for {duration}s...')

        start_time = time.time()
        rate = self.create_rate(50)  # 50 Hz

        while time.time() - start_time < duration:
            t = time.time() - start_time
            angle = np.sin(2 * np.pi * frequency * t) * 0.5

            
            left_pos = [angle, 0.3, 0.0, -1.2, 0.0, 1.5, angle]
            left_msg = Float64MultiArray()
            left_msg.data = left_pos

            
            right_pos = [-angle, -0.3, 0.0, -1.2, 0.0, 1.5, -angle]
            right_msg = Float64MultiArray()
            right_msg.data = right_pos

            self.left_cmd_pub.publish(left_msg)
            self.right_cmd_pub.publish(right_msg)

            rate.sleep()

    def send_coordinated_motion(self, duration=5.0):
        
        self.get_logger().info(f'Performing coordinated motion for {duration}s...')

        start_time = time.time()
        rate = self.create_rate(50)  # 50 Hz

        while time.time() - start_time < duration:
            t = time.time() - start_time
            phase = (t / duration) * 2 * np.pi

            
            q1 = np.sin(phase) * 0.5
            q2 = np.cos(phase) * 0.3

            left_pos = [q1, q2, 0.0, -1.57, 0.0, 1.57, -0.785]
            right_pos = [q1, -q2, 0.0, -1.57, 0.0, 1.57, -0.785]

            left_msg = Float64MultiArray()
            left_msg.data = left_pos

            right_msg = Float64MultiArray()
            right_msg.data = right_pos

            self.left_cmd_pub.publish(left_msg)
            self.right_cmd_pub.publish(right_msg)

            rate.sleep()

    def run_demo(self):
        
        self.get_logger().info('Starting demonstration sequence...')

       
        self.send_home_position()
        time.sleep(2.0)

        
        self.send_wave_motion(duration=5.0, frequency=0.5)
        time.sleep(1.0)

        
        self.send_coordinated_motion(duration=5.0)
        time.sleep(1.0)

        
        self.send_home_position()
        time.sleep(2.0)

        self.get_logger().info('Demonstration complete!')


def main(args=None):
    print("=" * 60)
    print("Dual Franka Test Controller")
    print("=" * 60)

    rclpy.init(args=args)

    try:
        controller = TestController()

        print("\nRunning demonstration sequence...")
        print("The arms will:")
        print("  1. Move to home position")
        print("  2. Perform waving motion")
        print("  3. Perform coordinated motion")
        print("  4. Return to home position")
        print("\nPress Ctrl+C to stop at any time.")
        print("=" * 60 + "\n")

        controller.run_demo()

        print("\nDemo complete! Keeping node alive for manual commands...")
        print("You can publish to /left_arm/joint_commands or /right_arm/joint_commands")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()
