#!/usr/bin/env python3
"""
Test script to verify installation of all dependencies
"""

import sys

def test_imports():
    """Test if all required packages can be imported"""
    results = []

    # Test MuJoCo
    try:
        import mujoco
        results.append(("MuJoCo", True, mujoco.__version__))
    except ImportError as e:
        results.append(("MuJoCo", False, str(e)))

    # Test NumPy
    try:
        import numpy
        results.append(("NumPy", True, numpy.__version__))
    except ImportError as e:
        results.append(("NumPy", False, str(e)))

    # Test ROS2 rclpy
    try:
        import rclpy
        results.append(("rclpy (ROS2)", True, "OK"))
    except ImportError as e:
        results.append(("rclpy (ROS2)", False, str(e)))

    # Test sensor_msgs
    try:
        import sensor_msgs
        results.append(("sensor_msgs", True, "OK"))
    except ImportError as e:
        results.append(("sensor_msgs", False, str(e)))

    # Test std_msgs
    try:
        import std_msgs
        results.append(("std_msgs", True, "OK"))
    except ImportError as e:
        results.append(("std_msgs", False, str(e)))

    return results


def main():
    print("=" * 60)
    print("Dual Franka MuJoCo + ROS2 Installation Test")
    print("=" * 60)
    print()

    results = test_imports()

    all_ok = True
    for name, success, info in results:
        status = "✓ OK" if success else "✗ FAILED"
        print(f"{status:10} {name:20} {info}")
        if not success:
            all_ok = False

    print()
    print("=" * 60)

    if all_ok:
        print("All dependencies are installed correctly!")
        print("You can now run the simulation.")
        print()
        print("Next steps:")
        print("  1. Run: python3 mujoco_dual_franka_sim.py")
        print("  or")
        print("  2. Run: ./run_simulation.sh")
        return 0
    else:
        print("Some dependencies are missing!")
        print()
        print("To install missing dependencies:")
        print("  - MuJoCo: pip3 install mujoco")
        print("  - NumPy: pip3 install numpy")
        print("  - ROS2: Follow https://docs.ros.org/en/humble/Installation.html")
        print()
        print("Or run the installation script:")
        print("  ./install_dependencies.sh")
        return 1

    print("=" * 60)


if __name__ == "__main__":
    sys.exit(main())
