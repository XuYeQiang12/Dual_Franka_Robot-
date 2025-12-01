#!/usr/bin/env python3


import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import threading
import time


class DualFrankaSimulator(Node):
    def __init__(self):
        super().__init__('dual_franka_simulator')


        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'dual_franka_scene.xml')
        if not os.path.exists(model_path):
            model_path = '/home/xyq/Documents/pythoncode/Franka_code/dual_franka_scene.xml'
        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info('MuJoCo model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load MuJoCo model: {e}')
            raise


        self.left_joint_names = [
            'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
            'left_joint5', 'left_joint6', 'left_joint7'
        ]
        self.right_joint_names = [
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
            'right_joint5', 'right_joint6', 'right_joint7'
        ]


        self.left_joint_ids = [self.model.joint(name).id for name in self.left_joint_names]
        self.right_joint_ids = [self.model.joint(name).id for name in self.right_joint_names]


        self.left_actuator_ids = [i for i in range(7)]
        self.right_actuator_ids = [i for i in range(7, 14)]

        self.left_joint_state_pub = self.create_publisher(
            JointState, '/left_arm/joint_states', 10)
        self.right_joint_state_pub = self.create_publisher(
            JointState, '/right_arm/joint_states', 10)


        self.left_joint_cmd_sub = self.create_subscription(
            Float64MultiArray, '/left_arm/joint_commands',
            self.left_joint_cmd_callback, 10)
        self.right_joint_cmd_sub = self.create_subscription(
            Float64MultiArray, '/right_arm/joint_commands',
            self.right_joint_cmd_callback, 10)


        self.left_target_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        self.right_target_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])


        self.data.qpos[:7] = self.left_target_pos
        self.data.qpos[7:14] = self.right_target_pos


        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz

        self.running = True
        self.sim_thread = None

        self.get_logger().info('Dual Franka Simulator initialized')
        self.get_logger().info(f'Left arm joints: {self.left_joint_names}')
        self.get_logger().info(f'Right arm joints: {self.right_joint_names}')

    def left_joint_cmd_callback(self, msg):
        """Callback for left arm joint commands"""
        if len(msg.data) == 7:
            self.left_target_pos = np.array(msg.data)
            self.get_logger().debug(f'Received left arm command: {self.left_target_pos}')
        else:
            self.get_logger().warn(f'Invalid left arm command size: {len(msg.data)}, expected 7')

    def right_joint_cmd_callback(self, msg):
        """Callback for right arm joint commands"""
        if len(msg.data) == 7:
            self.right_target_pos = np.array(msg.data)
            self.get_logger().debug(f'Received right arm command: {self.right_target_pos}')
        else:
            self.get_logger().warn(f'Invalid right arm command size: {len(msg.data)}, expected 7')

    def publish_joint_states(self):
        """Publish current joint states for both arms"""
        current_time = self.get_clock().now().to_msg()

      
        left_msg = JointState()
        left_msg.header.stamp = current_time
        left_msg.header.frame_id = 'left_base'
        left_msg.name = self.left_joint_names
        left_msg.position = self.data.qpos[:7].tolist()
        left_msg.velocity = self.data.qvel[:7].tolist()
        self.left_joint_state_pub.publish(left_msg)

      
        right_msg = JointState()
        right_msg.header.stamp = current_time
        right_msg.header.frame_id = 'right_base'
        right_msg.name = self.right_joint_names
        right_msg.position = self.data.qpos[7:14].tolist()
        right_msg.velocity = self.data.qvel[7:14].tolist()
        self.right_joint_state_pub.publish(right_msg)

    def simulation_step(self):
        
        self.data.ctrl[:7] = self.left_target_pos
        self.data.ctrl[7:14] = self.right_target_pos

        
        mujoco.mj_step(self.model, self.data)

    def run_simulation_with_viewer(self):
        

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0, 0, 0.8]

            while self.running and viewer.is_running():
                step_start = time.time()

                
                self.simulation_step()

                
                viewer.sync()

               
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        self.get_logger().info('MuJoCo viewer closed')

    def run_simulation_headless(self):
        
        self.get_logger().info('Running in headless mode...')

        while self.running:
            step_start = time.time()

            
            self.simulation_step()

            
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def start_simulation(self, use_viewer=True):
        
        if use_viewer:
            self.sim_thread = threading.Thread(target=self.run_simulation_with_viewer)
        else:
            self.sim_thread = threading.Thread(target=self.run_simulation_headless)

        self.sim_thread.start()
        self.get_logger().info('Simulation thread started')

    def stop_simulation(self):
        
        self.running = False
        if self.sim_thread:
            self.sim_thread.join()
        self.get_logger().info('Simulation stopped')


def main(args=None):
    print("=" * 60)
    print("Dual Franka Robot Simulator with ROS2")
    print("=" * 60)

    # 初始化 ROS2
    rclpy.init(args=args)

    try:
        # 创建节点
        simulator = DualFrankaSimulator()

        # 启动模拟器（带可视化界面）
        simulator.start_simulation(use_viewer=True)

        
        print("\nSimulation running. Press Ctrl+C to exit.")
        print("\nROS2 Topics:")
        print("  Publishers:")
        print("    /left_arm/joint_states  - Left arm joint states")
        print("    /right_arm/joint_states - Right arm joint states")
        print("  Subscribers:")
        print("    /left_arm/joint_commands  - Left arm joint position commands")
        print("    /right_arm/joint_commands - Right arm joint position commands")
        print("\nExample command to move left arm:")
        print("  ros2 topic pub --once /left_arm/joint_commands std_msgs/msg/Float64MultiArray")
        print("    '{data: [0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0]}'")
        print("=" * 60)

        rclpy.spin(simulator)

    except KeyboardInterrupt:
        print("\nShutting down...")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        if 'simulator' in locals():
            simulator.stop_simulation()
            simulator.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()
