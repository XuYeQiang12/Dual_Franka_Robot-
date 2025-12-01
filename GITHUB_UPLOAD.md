# 如何上传项目到GitHub

本文档说明如何将这个双臂Franka机器人仿真项目上传到GitHub。

## 准备工作

### 1. 配置Git用户信息

如果您是第一次使用Git,需要配置用户信息:

```bash
git config --global user.name "您的姓名"
git config --global user.email "your.email@example.com"
```

查看当前配置:
```bash
git config user.name
git config user.email
```

### 2. 检查Git状态

项目已经初始化为Git仓库,检查状态:

```bash
git status
```

## 上传步骤

### 步骤1: 创建初始提交

如果还没有提交,运行:

```bash
# 添加所有文件
git add .

# 创建提交
git commit -m "Initial commit: Dual Franka MuJoCo Simulation with ROS2"
```

### 步骤2: 在GitHub上创建新仓库

1. 访问 GitHub: https://github.com/new
2. 填写仓库信息:
   - **Repository name**: `dual-franka-mujoco-ros2` (或您喜欢的名称)
   - **Description**: `Dual Franka FR3 Robot Simulation with MuJoCo and ROS2`
   - **Public** 或 **Private**: 根据需要选择
   - ⚠️ **不要勾选** "Initialize this repository with a README"
   - ⚠️ **不要添加** .gitignore 或 license (我们已经有了)
3. 点击 **Create repository**

### 步骤3: 连接本地仓库到GitHub

创建仓库后,GitHub会显示命令。使用HTTPS方式:

```bash
# 添加远程仓库 (替换YOUR_USERNAME为您的GitHub用户名)
git remote add origin https://github.com/YOUR_USERNAME/dual-franka-mujoco-ros2.git

# 重命名分支为main
git branch -M main

# 推送到GitHub
git push -u origin main
```

### 步骤4: 验证上传

访问您的GitHub仓库页面,应该能看到所有文件。

## 使用SSH方式 (可选,更安全)

如果您配置了SSH密钥:

```bash
# 使用SSH URL
git remote add origin git@github.com:YOUR_USERNAME/dual-franka-mujoco-ros2.git
git branch -M main
git push -u origin main
```

### 配置SSH密钥

如果还没有SSH密钥:

1. 生成SSH密钥:
```bash
ssh-keygen -t ed25519 -C "your.email@example.com"
```

2. 复制公钥:
```bash
cat ~/.ssh/id_ed25519.pub
```

3. 添加到GitHub:
   - 访问: https://github.com/settings/keys
   - 点击 "New SSH key"
   - 粘贴公钥内容
   - 点击 "Add SSH key"

## 后续更新

### 推送新的更改

修改代码后:

```bash
# 查看修改
git status

# 添加修改的文件
git add .

# 提交
git commit -m "描述您的修改"

# 推送到GitHub
git push
```

### 查看提交历史

```bash
git log --oneline
```

### 创建分支进行开发

```bash
# 创建并切换到新分支
git checkout -b feature/new-feature

# 开发完成后推送分支
git push -u origin feature/new-feature
```

## 项目README预览

上传后,GitHub会自动显示 README.md 作为项目首页,包含:
- 项目介绍
- 安装说明
- 使用方法
- ROS2接口
- 示例代码

## 添加GitHub徽章 (可选)

可以在README.md开头添加徽章:

```markdown
# Dual Franka MuJoCo Simulation

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)
![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
```

## 故障排除

### 推送被拒绝

如果看到错误 "Updates were rejected":

```bash
# 先拉取远程更改
git pull origin main --rebase

# 再推送
git push
```

### 忘记提交就推送

```bash
# 撤销到上一次提交
git reset HEAD~1

# 重新添加和提交
git add .
git commit -m "正确的提交信息"
git push
```

### 修改最后一次提交信息

```bash
git commit --amend -m "新的提交信息"
git push --force-with-lease
```

## 常用Git命令速查

```bash
# 查看状态
git status

# 查看差异
git diff

# 查看日志
git log

# 撤销修改(未暂存)
git checkout -- filename

# 撤销添加(已暂存)
git reset HEAD filename

# 查看远程仓库
git remote -v

# 同步远程更改
git pull

# 克隆仓库
git clone https://github.com/YOUR_USERNAME/dual-franka-mujoco-ros2.git
```

## 项目结构说明

上传到GitHub的文件包括:

```
dual-franka-mujoco-ros2/
├── .gitignore                      # Git忽略文件
├── README.md                       # 项目文档(英文)
├── QUICKSTART.md                   # 快速开始(中文)
├── PROJECT_STRUCTURE.txt           # 项目结构说明
├── GITHUB_UPLOAD.md               # 本文件
├── dual_franka_scene.xml          # MuJoCo场景
├── mujoco_dual_franka_sim.py      # 主仿真程序
├── demo_no_ros.py                 # 独立演示
├── test_installation.py           # 依赖测试
├── install_dependencies.sh        # 安装脚本
├── install_python_deps.sh         # Python依赖安装
├── run_simulation.sh              # 启动脚本
├── setup_git.sh                   # Git配置脚本
└── dual_franka_ros2/              # ROS2功能包
    ├── package.xml
    ├── setup.py
    └── dual_franka_ros2/
        ├── __init__.py
        ├── mujoco_dual_franka_sim.py
        ├── test_controller.py
        └── dual_franka_scene.xml
```

## 推荐的仓库设置

上传后,在GitHub仓库设置中:

1. **About** (仓库描述):
   - 添加描述: "Dual Franka FR3 Robot Simulation with MuJoCo and ROS2"
   - 添加话题: `robotics`, `mujoco`, `ros2`, `franka`, `simulation`

2. **Topics** (标签):
   - robotics
   - ros2
   - mujoco
   - franka-robot
   - simulation
   - dual-arm

3. **Issues**: 启用Issues功能便于反馈

4. **Wiki**: 可选,用于更详细的文档

## 示例GitHub仓库URL

上传成功后,您的仓库地址将是:

```
https://github.com/YOUR_USERNAME/dual-franka-mujoco-ros2
```

其他人可以通过以下命令克隆:

```bash
git clone https://github.com/YOUR_USERNAME/dual-franka-mujoco-ros2.git
cd dual-franka-mujoco-ros2
./install_dependencies.sh
python3 demo_no_ros.py
```

## 获取帮助

- Git官方文档: https://git-scm.com/doc
- GitHub指南: https://docs.github.com/
- Git中文教程: https://www.liaoxuefeng.com/wiki/896043488029600

---

完成上传后,欢迎分享您的GitHub仓库链接!
