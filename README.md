# Dual Franka Robot MuJoCo Simulation with ROS2

This project provides a complete dual-arm Franka FR3 robot simulation using MuJoCo with ROS2 integration for communication and control.
<img width="2496" height="1389" alt="image" src="https://github.com/user-attachments/assets/e9c27099-630a-464f-b7d8-1112b2eabb62" />

## Features

- **Dual-Arm Setup**: Two Franka FR3 robots positioned side-by-side
- **MuJoCo Physics**: High-fidelity physics simulation
- **ROS2 Integration**: Full ROS2 communication for control and monitoring
- **Real-time Visualization**: Interactive MuJoCo viewer
- **Position Control**: Direct joint position control for both arms
- **Joint State Publishing**: Real-time joint state feedback at 100 Hz

## System Requirements

- Ubuntu 20.04 or 22.04
- Python 3.8+
- ROS2 (Humble recommended)
- MuJoCo 3.0+

## Installation

### Quick Installation

Run the automated installation script:

```bash
cd /home/xyq/Documents/pythoncode/Franka_code
./install_dependencies.sh
```

### Manual Installation

1. **Install MuJoCo**:
```bash
pip3 install mujoco
```

2. **Install ROS2 Humble** (if not already installed):
Follow the official guide: https://docs.ros.org/en/humble/Installation.html

3. **Install Python dependencies**:
```bash
pip3 install numpy rclpy
```

## Usage

### Method 1: Direct Python Execution (Recommended for Testing)

Simply run the simulation script directly:

```bash
cd /home/xyq/Documents/pythoncode/Franka_code
python3 mujoco_dual_franka_sim.py
```

Or use the convenience script:

```bash
./run_simulation.sh
```

### Method 2: ROS2 Package Build

1. **Build the ROS2 package**:
```bash
cd /home/xyq/Documents/pythoncode/Franka_code/dual_franka_ros2
colcon build
```

2. **Source the workspace**:
```bash
source install/setup.bash
```

3. **Run the simulation**:
```bash
ros2 run dual_franka_ros2 dual_franka_sim
```

4. **Run the test controller** (in a new terminal):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dual_franka_ros2 test_controller
```

## ROS2 Topics

### Published Topics

- `/left_arm/joint_states` (sensor_msgs/JointState)
  - Left arm joint positions, velocities
  - Published at 100 Hz

- `/right_arm/joint_states` (sensor_msgs/JointState)
  - Right arm joint positions, velocities
  - Published at 100 Hz

### Subscribed Topics

- `/left_arm/joint_commands` (std_msgs/Float64MultiArray)
  - Target joint positions for left arm (7 values)

- `/right_arm/joint_commands` (std_msgs/Float64MultiArray)
  - Target joint positions for right arm (7 values)

## Controlling the Robots

### Using Command Line

Move the left arm:
```bash
ros2 topic pub --once /left_arm/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0]}"
```

Move the right arm:
```bash
ros2 topic pub --once /right_arm/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.5, 0.0, -1.0, 0.0, 1.5, 0.0]}"
```

### Using Python Script

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('test_publisher')
pub = node.create_publisher(Float64MultiArray, '/left_arm/joint_commands', 10)

msg = Float64MultiArray()
msg.data = [0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0]
pub.publish(msg)
```

### Joint Limits

Each arm has 7 joints with the following limits (in radians):

- Joint 1: [-2.7437, 2.7437]
- Joint 2: [-1.7837, 1.7837]
- Joint 3: [-2.9007, 2.9007]
- Joint 4: [-3.0421, -0.1518]
- Joint 5: [-2.8065, 2.8065]
- Joint 6: [0.5445, 4.5169]
- Joint 7: [-3.0159, 3.0159]

## Project Structure

```
Franka_code/
├── dual_franka_scene.xml          # MuJoCo scene with dual Franka arms
├── mujoco_dual_franka_sim.py      # Main simulation script
├── install_dependencies.sh        # Automated installation script
├── run_simulation.sh              # Quick launch script
├── README.md                      # This file
└── dual_franka_ros2/              # ROS2 package
    ├── package.xml
    ├── setup.py
    ├── resource/
    └── dual_franka_ros2/
        ├── __init__.py
        ├── mujoco_dual_franka_sim.py
        ├── test_controller.py
        └── dual_franka_scene.xml
```

## Troubleshooting

### MuJoCo model not found
Make sure the Franka FR3 model path is correct in `dual_franka_scene.xml`:
```xml
<compiler angle="radian" meshdir="/home/xyq/mujoco_sim/models/mujoco_menagerie/franka_fr3/assets"/>
```

### ROS2 not found
Source ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Module 'mujoco' not found
Install MuJoCo:
```bash
pip3 install mujoco
```

### Simulation runs but no viewer appears
Check if you're running in a headless environment. The viewer requires a display.

### Joint commands not working
- Ensure the simulation is running
- Check topic names with `ros2 topic list`
- Verify message format with `ros2 topic echo /left_arm/joint_commands`

## Advanced Usage

### Custom Control Algorithms

You can implement custom control algorithms by subscribing to joint states and publishing commands:

```python
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')
        self.sub = self.create_subscription(
            JointState, '/left_arm/joint_states',
            self.joint_state_callback, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/left_arm/joint_commands', 10)

    def joint_state_callback(self, msg):
        # Your control algorithm here
        cmd = Float64MultiArray()
        cmd.data = [...]  # Computed joint positions
        self.pub.publish(cmd)
```

### Trajectory Execution

Send smooth trajectories by publishing at high frequency:

```python
import numpy as np
import time

rate = node.create_rate(100)  # 100 Hz
for t in np.linspace(0, 5, 500):
    positions = compute_trajectory(t)
    msg.data = positions
    pub.publish(msg)
    rate.sleep()
```

## References

- MuJoCo Documentation: https://mujoco.readthedocs.io/
- ROS2 Humble Documentation: https://docs.ros.org/en/humble/
- Franka FR3 Robot: https://www.franka.de/

## License

MIT License

## Author

Your Name

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.
