# 双臂Franka机器人MuJoCo仿真 - 快速开始指南

## 概述

本项目实现了双臂Franka FR3机器人的MuJoCo物理仿真,并集成了ROS2通信。

## 系统要求

- Ubuntu 20.04 或 22.04
- Python 3.8+
- MuJoCo 3.0+
- ROS2 Humble (可选,用于ROS2通信)

## 快速安装

### 方法1: 自动安装(推荐)

```bash
cd /home/xyq/Documents/pythoncode/Franka_code
./install_dependencies.sh
```

### 方法2: 手动安装

1. 安装MuJoCo:
```bash
pip3 install mujoco
```

2. 安装NumPy:
```bash
pip3 install numpy
```

3. (可选) 安装ROS2 Humble:
参考官方文档: https://docs.ros.org/en/humble/Installation.html

## 测试安装

运行测试脚本检查依赖:

```bash
python3 test_installation.py
```

## 运行仿真

### 选项1: 独立演示(不需要ROS2)

最简单的方式,直接运行:

```bash
python3 demo_no_ros.py
```

这个版本会自动演示机器人的协调运动,适合快速测试。

### 选项2: 带ROS2通信的完整仿真

需要先安装ROS2,然后:

```bash
# 终端1: 运行仿真
source /opt/ros/humble/setup.bash
python3 mujoco_dual_franka_sim.py
```

```bash
# 终端2: 运行测试控制器
source /opt/ros/humble/setup.bash
cd dual_franka_ros2
colcon build
source install/setup.bash
ros2 run dual_franka_ros2 test_controller
```

### 选项3: 使用快捷脚本

```bash
./run_simulation.sh
```

## ROS2话题说明

### 发布的话题 (仿真发布给外部)

- `/left_arm/joint_states` - 左臂关节状态 (100 Hz)
- `/right_arm/joint_states` - 右臂关节状态 (100 Hz)

### 订阅的话题 (外部发送给仿真)

- `/left_arm/joint_commands` - 左臂关节位置命令
- `/right_arm/joint_commands` - 右臂关节位置命令

## 手动控制示例

### 命令行控制

移动左臂:
```bash
ros2 topic pub --once /left_arm/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0]}"
```

移动右臂到Home位置:
```bash
ros2 topic pub --once /right_arm/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]}"
```

### Python控制示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('my_controller')

# 创建发布者
left_pub = node.create_publisher(Float64MultiArray, '/left_arm/joint_commands', 10)

# 发送命令
msg = Float64MultiArray()
msg.data = [0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0]
left_pub.publish(msg)

print("命令已发送!")
```

## 关节限制

每个机械臂有7个关节,限制如下(单位:弧度):

| 关节 | 最小值 | 最大值 |
|------|--------|--------|
| Joint 1 | -2.7437 | 2.7437 |
| Joint 2 | -1.7837 | 1.7837 |
| Joint 3 | -2.9007 | 2.9007 |
| Joint 4 | -3.0421 | -0.1518 |
| Joint 5 | -2.8065 | 2.8065 |
| Joint 6 | 0.5445 | 4.5169 |
| Joint 7 | -3.0159 | 3.0159 |

## 项目文件说明

```
Franka_code/
├── dual_franka_scene.xml          # MuJoCo场景文件 (双臂机器人定义)
├── mujoco_dual_franka_sim.py      # 主仿真程序 (带ROS2)
├── demo_no_ros.py                 # 独立演示程序 (不需要ROS2)
├── test_installation.py           # 依赖测试脚本
├── install_dependencies.sh        # 自动安装脚本
├── run_simulation.sh              # 快速启动脚本
├── README.md                      # 详细文档
├── QUICKSTART.md                  # 本文件 (快速开始指南)
└── dual_franka_ros2/              # ROS2功能包
    ├── package.xml                # ROS2包配置
    ├── setup.py                   # Python包配置
    └── dual_franka_ros2/
        ├── mujoco_dual_franka_sim.py   # 仿真节点
        ├── test_controller.py          # 测试控制器
        └── dual_franka_scene.xml       # 场景文件副本
```

## 常见问题

### 1. MuJoCo模型加载失败

**问题**: 提示找不到mesh文件

**解决**:
- 确认Franka FR3模型路径正确: `/home/xyq/mujoco_sim/models/mujoco_menagerie/franka_fr3`
- 检查 `dual_franka_scene.xml` 中的 `meshdir` 路径是否正确

### 2. 找不到mujoco模块

**问题**: `ModuleNotFoundError: No module named 'mujoco'`

**解决**:
```bash
pip3 install mujoco
```

### 3. ROS2未找到

**问题**: 提示 `ros2` 命令不存在

**解决**:
```bash
source /opt/ros/humble/setup.bash
```
或安装ROS2: https://docs.ros.org/en/humble/Installation.html

### 4. 仿真运行但没有显示窗口

**问题**: 在无图形界面环境运行

**解决**: 需要在有显示器的环境中运行,或使用VNC/X11转发

### 5. 关节命令无效

**问题**: 发送命令后机器人不动

**检查**:
- 确认仿真正在运行
- 检查话题名称: `ros2 topic list`
- 检查消息格式: `ros2 topic echo /left_arm/joint_commands`

## 下一步

1. 查看完整文档: [README.md](README.md)
2. 开发自定义控制器
3. 实现轨迹规划
4. 添加末端执行器
5. 集成视觉传感器

## 技术支持

- MuJoCo文档: https://mujoco.readthedocs.io/
- ROS2文档: https://docs.ros.org/en/humble/
- Franka机器人: https://www.franka.de/

## 示例使用场景

### 场景1: 双臂协作抓取

```python
# 左臂移动到抓取位置
left_cmd.data = [0.5, 0.3, 0.2, -1.2, 0.0, 1.5, 0.0]

# 右臂移动到辅助位置
right_cmd.data = [-0.5, -0.3, -0.2, -1.2, 0.0, 1.5, 0.0]
```

### 场景2: 镜像运动

```python
# 左右臂镜像运动
angle = 0.5
left_cmd.data = [angle, 0.3, 0.0, -1.5, 0.0, 1.5, 0.0]
right_cmd.data = [angle, -0.3, 0.0, -1.5, 0.0, 1.5, 0.0]
```

### 场景3: 轨迹跟踪

```python
import numpy as np

# 生成圆形轨迹
for t in np.linspace(0, 2*np.pi, 100):
    q1 = 0.5 * np.sin(t)
    q2 = 0.3 * np.cos(t)

    left_cmd.data = [q1, q2, 0.0, -1.57, 0.0, 1.57, 0.0]
    left_pub.publish(left_cmd)
    time.sleep(0.02)  # 50 Hz
```

---

祝您使用愉快!如有问题,请参考完整的 [README.md](README.md) 文档。
