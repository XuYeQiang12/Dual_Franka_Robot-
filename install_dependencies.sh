#!/bin/bash

# Installation script for Dual Franka MuJoCo Simulation with ROS2

set -e

echo "=========================================="
echo "Dual Franka MuJoCo + ROS2 Installation"
echo "=========================================="

# Check if running on Ubuntu
if [[ ! -f /etc/os-release ]]; then
    echo "Error: Cannot detect OS"
    exit 1
fi

source /etc/os-release

echo ""
echo "Step 1: Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev

echo ""
echo "Step 2: Installing Python dependencies..."
pip3 install --user mujoco
pip3 install --user numpy

echo ""
echo "Step 3: Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    echo "Warning: ROS2 not found!"
    echo "Please install ROS2 (Humble recommended) from:"
    echo "https://docs.ros.org/en/humble/Installation.html"
    echo ""
    read -p "Do you want to install ROS2 Humble now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Installing ROS2 Humble..."

        # Add ROS2 repository
        sudo apt-get install -y software-properties-common
        sudo add-apt-repository universe
        sudo apt-get update
        sudo apt-get install -y curl

        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        sudo apt-get update
        sudo apt-get install -y ros-humble-desktop
        sudo apt-get install -y ros-humble-ros-base
        sudo apt-get install -y python3-colcon-common-extensions

        # Setup environment
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        source /opt/ros/humble/setup.bash

        echo "ROS2 Humble installed successfully!"
    else
        echo "Skipping ROS2 installation. Please install it manually."
    fi
else
    echo "ROS2 found: $(ros2 --version)"
fi

echo ""
echo "Step 4: Verifying MuJoCo installation..."
python3 -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')" || {
    echo "Error: MuJoCo import failed!"
    echo "Please ensure mujoco is installed: pip3 install mujoco"
    exit 1
}

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source ROS2: source /opt/ros/humble/setup.bash"
echo "2. Build the package: cd dual_franka_ros2 && colcon build"
echo "3. Source the workspace: source install/setup.bash"
echo "4. Run the simulation: ros2 run dual_franka_ros2 dual_franka_sim"
echo ""
echo "For more information, see README.md"
echo "=========================================="
