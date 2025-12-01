#!/usr/bin/env python3
"""
MuJoCo Dual Franka Robot Simulation - Standalone Demo (No ROS2)
This version runs without ROS2 for quick testing
"""

import numpy as np
import mujoco
import mujoco.viewer
import time


def main():
    print("=" * 60)
    print("Dual Franka Robot Simulation - Standalone Demo")
    print("=" * 60)

    # Load MuJoCo model
    model_path = '/home/xyq/Documents/pythoncode/Franka_code/dual_franka_scene.xml'
    print(f'\nLoading MuJoCo model from: {model_path}')

    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print('MuJoCo model loaded successfully!')
    except Exception as e:
        print(f'ERROR: Failed to load MuJoCo model: {e}')
        print('\nPlease ensure:')
        print('1. MuJoCo is installed: pip3 install mujoco')
        print('2. The Franka FR3 model path is correct in dual_franka_scene.xml')
        return 1

    # Joint names
    left_joints = ['left_joint' + str(i) for i in range(1, 8)]
    right_joints = ['right_joint' + str(i) for i in range(1, 8)]

    print(f'\nLeft arm joints: {left_joints}')
    print(f'Right arm joints: {right_joints}')

    # Set initial position to home
    home_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
    data.qpos[:7] = home_pos
    data.qpos[7:14] = home_pos

    print('\nStarting simulation with demo motions...')
    print('\nControls in the viewer:')
    print('  - Left click + drag: Rotate view')
    print('  - Right click + drag: Pan view')
    print('  - Scroll: Zoom')
    print('  - ESC or close window: Exit')
    print('\nThe robots will perform coordinated motions automatically.')
    print("=" * 60)

    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set camera to get a good view
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.distance = 3.0
        viewer.cam.lookat[:] = [0, 0, 0.8]

        start_time = time.time()
        phase = 0  # 0: home, 1: wave, 2: coordinated

        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - start_time

            # Demo motion sequence
            if elapsed < 3:
                # Phase 0: Stay at home
                target_left = home_pos.copy()
                target_right = home_pos.copy()

            elif elapsed < 8:
                # Phase 1: Wave motion
                t = elapsed - 3
                wave = np.sin(2 * np.pi * 0.5 * t) * 0.5

                target_left = np.array([wave, 0.3, 0, -1.2, 0, 1.5, wave])
                target_right = np.array([-wave, -0.3, 0, -1.2, 0, 1.5, -wave])

            elif elapsed < 13:
                # Phase 2: Coordinated motion
                t = elapsed - 8
                angle = (t / 5.0) * 2 * np.pi
                q1 = np.sin(angle) * 0.5
                q2 = np.cos(angle) * 0.3

                target_left = np.array([q1, q2, 0, -1.57, 0, 1.57, -0.785])
                target_right = np.array([q1, -q2, 0, -1.57, 0, 1.57, -0.785])

            else:
                # Reset to home and restart
                target_left = home_pos.copy()
                target_right = home_pos.copy()
                if elapsed > 16:
                    start_time = time.time()

            # Set control inputs
            data.ctrl[:7] = target_left
            data.ctrl[7:14] = target_right

            # Step simulation
            mujoco.mj_step(model, data)

            # Sync viewer
            viewer.sync()

            # Print joint states every 2 seconds
            if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.01:
                print(f"\n[{elapsed:.1f}s] Joint positions:")
                print(f"  Left arm:  {data.qpos[:7]}")
                print(f"  Right arm: {data.qpos[7:14]}")

            # Maintain real-time execution
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("\nSimulation ended.")
    return 0


if __name__ == '__main__':
    try:
        exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
        exit(0)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
