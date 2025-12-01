#!/bin/bash

# Git配置和GitHub上传指南

echo "=========================================="
echo "Git配置和GitHub上传指南"
echo "=========================================="
echo ""

# 检查是否已配置Git
if git config user.name >/dev/null 2>&1; then
    echo "✓ Git用户名已配置: $(git config user.name)"
else
    echo "请配置您的Git用户名:"
    read -p "输入您的姓名: " username
    git config --global user.name "$username"
    echo "✓ 用户名已设置: $username"
fi

if git config user.email >/dev/null 2>&1; then
    echo "✓ Git邮箱已配置: $(git config user.email)"
else
    echo "请配置您的Git邮箱:"
    read -p "输入您的邮箱: " email
    git config --global user.email "$email"
    echo "✓ 邮箱已设置: $email"
fi

echo ""
echo "=========================================="
echo "上传到GitHub的步骤"
echo "=========================================="
echo ""
echo "1. 在GitHub上创建新仓库:"
echo "   - 访问: https://github.com/new"
echo "   - 仓库名建议: dual-franka-mujoco-ros2"
echo "   - 描述: Dual Franka FR3 Robot Simulation with MuJoCo and ROS2"
echo "   - 不要勾选 'Initialize with README' (我们已经有了)"
echo ""
echo "2. 创建初始提交 (如果还没有):"
echo "   git add ."
echo "   git commit -m 'Initial commit: Dual Franka MuJoCo + ROS2'"
echo ""
echo "3. 连接到GitHub仓库 (替换YOUR_USERNAME为您的GitHub用户名):"
echo "   git remote add origin https://github.com/YOUR_USERNAME/dual-franka-mujoco-ros2.git"
echo ""
echo "4. 推送代码到GitHub:"
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "=========================================="
echo ""
echo "当前Git状态:"
git status
echo ""
echo "=========================================="
