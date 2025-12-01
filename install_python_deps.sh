#!/bin/bash

# Simple Python dependencies installation (no sudo required)

set -e

echo "=========================================="
echo "Installing Python Dependencies"
echo "=========================================="

echo ""
echo "Installing MuJoCo..."
pip3 install --user mujoco

echo ""
echo "Installing NumPy..."
pip3 install --user numpy

echo ""
echo "Verifying installations..."
python3 -c "import mujoco; print(f'✓ MuJoCo {mujoco.__version__} installed')"
python3 -c "import numpy; print(f'✓ NumPy {numpy.__version__} installed')"

echo ""
echo "=========================================="
echo "Python dependencies installed!"
echo "=========================================="
echo ""
echo "To run the demo without ROS2:"
echo "  python3 demo_no_ros.py"
echo ""
echo "For ROS2 features, you need to install ROS2 separately:"
echo "  https://docs.ros.org/en/humble/Installation.html"
echo "=========================================="
