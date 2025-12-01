#!/bin/bash

# Quick launch script for Dual Franka Simulation

echo "=========================================="
echo "Launching Dual Franka MuJoCo Simulation"
echo "=========================================="

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        echo "Error: Could not source ROS2. Please install ROS2 first."
        exit 1
    }
fi

# Check if running from package directory or main directory
if [ -d "dual_franka_ros2/install" ]; then
    echo "Sourcing workspace from dual_franka_ros2/install..."
    source dual_franka_ros2/install/setup.bash
elif [ -d "install" ]; then
    echo "Sourcing workspace from install..."
    source install/setup.bash
fi

# Run the simulation directly with Python
echo "Starting simulation..."
echo ""

cd "$(dirname "$0")"
python3 mujoco_dual_franka_sim.py

echo ""
echo "Simulation ended."
