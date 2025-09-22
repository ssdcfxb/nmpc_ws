# Auto Robot - 移动机械臂仿真平台

基于 ROS2 和 Gazebo 的移动机械臂仿真平台，集成了麦卡纳姆轮底盘和 Z1 机械臂。

## 项目概述

本项目包含：
- **移动底盘**: 配备四个麦卡纳姆轮的全向移动平台
- **Z1 机械臂**: 6自由度工业机械臂
- **传感器**: IMU、相机、激光雷达等
- **仿真环境**: Gazebo Ignition 仿真支持

## 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **仿真器**: Gazebo Ignition (gz-sim)
- **Python**: 3.10+

## 依赖安装

### ROS2 和 Gazebo

```bash
# 更新系统包
sudo apt update

# 安装 ROS2 Humble (如果尚未安装)
sudo apt install ros-humble-desktop

# 安装 Gazebo Ignition
sudo apt install gz-garden

# 安装 ROS2-Gazebo 桥接
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

### 必需的 ROS2 包

```bash
sudo apt install \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control \
    python3-colcon-common-extensions
```

## 构建项目

### 1. 克隆项目

```bash
# 创建工作空间
mkdir -p ~/nmpc_ws/src
cd ~/nmpc_ws/src

# 克隆项目
git clone https://github.com/ssdcfxb/nmpc_ws.git .
```

### 2. 安装依赖

```bash
# 返回工作空间根目录
cd ~/nmpc_ws

# 安装 rosdep 依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

```bash
# 编译所有包
colcon build

# 或者单独编译特定包
colcon build --packages-select auto_robot z1_description

# 设置环境变量
source install/setup.bash
```

## 运行仿真

### 1. 基础仿真启动

```bash
# 确保已经 source 环境
source ~/nmpc_ws/auto_remake_ws/install/setup.bash

# 启动完整仿真（包含 Gazebo + RViz）
ros2 launch auto_robot load_urdf_into_gazebo.launch.py
```

### 2. 仅显示 URDF 模型

```bash
# 仅在 RViz 中显示模型
ros2 launch auto_robot display_xacro.launch.py

# 或使用简单显示
ros2 launch auto_robot display.launch.py
```

### 3. 底盘控制测试

```bash
# 在新终端中测试底盘运动
source ~/nmpc_ws/auto_remake_ws/install/setup.bash

# 前进运动
# ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
#   "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
ign topic -t "/model/auto_robot/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.1, y: 0.0}"

# 侧向运动（麦卡纳姆轮特有）
# ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
#   "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
ign topic -t "/model/auto_robot/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.0, y: 0.1}"

# 旋转运动
# ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
#   "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" --once
ign topic -t "/model/auto_robot/cmd_vel" -m ignition.msgs.Twist -p "angular: {z: 0.1}"
```

## 项目结构

```
auto_remake_ws/
├── src/
│   ├── auto_robot/              # 主要机器人包
│   │   ├── config/              # 控制器配置
│   │   ├── launch/              # 启动文件
│   │   ├── meshes/              # 3D 模型文件
│   │   ├── robots/              # URDF/XACRO 文件
│   │   ├── rviz/                # RViz 配置
│   │   ├── urdf/                # URDF 组件
│   │   └── worlds/              # Gazebo 世界文件
│   └── z1_description/          # Z1 机械臂描述
│       ├── meshes/              # 机械臂网格文件
│       ├── urdf/                # 机械臂 URDF
│       └── xacro/               # 机械臂 XACRO 文件
└── README.md                    # 本文档
```

## 配置说明

### 1. 底盘配置

- **轮子类型**: 麦卡纳姆轮（已修改为球轮用于测试）
- **驱动方式**: Gazebo 麦卡纳姆插件
- **控制话题**: `/cmd_vel`
- **里程计话题**: `/odom`

### 2. 机械臂配置

- **模型**: Z1 6自由度机械臂
- **控制方式**: ros2_control 框架
- **配置文件**: `config/z1_controllers.yaml`

### 3. 传感器

- **IMU**: 惯性测量单元
- **相机**: RGB 相机
- **激光雷达**: 2D/3D 激光雷达支持

## 故障排除

### 1. 编译错误

```bash
# 清理编译缓存
rm -rf build/ install/ log/

# 重新编译
colcon build --packages-select auto_robot z1_description
```

### 2. Gazebo 显示问题

```bash
# 清理 Gazebo 缓存
rm -rf ~/.gz/ ~/.ignition/

# 重启仿真
ros2 launch auto_robot load_urdf_into_gazebo.launch.py
```

### 3. 模型不显示

```bash
# 检查 URDF 语法
xacro src/auto_robot/robots/auto_robot.xacro --check-order

# 验证生成的 URDF
xacro src/auto_robot/robots/auto_robot.xacro > /tmp/robot.urdf
```

### 4. 控制器问题

```bash
# 检查控制器状态
ros2 control list_controllers

# 检查话题
ros2 topic list | grep cmd_vel
ros2 topic list | grep odom
```

## 开发指南

### 1. 添加新传感器

1. 在 `urdf/sensor/` 下创建传感器文件
2. 在主 XACRO 文件中包含和实例化
3. 重新编译和测试

### 2. 修改轮子配置

1. 编辑 `urdf/wheel/wheel_*.urdf.xacro` 文件
2. 调整几何形状、材质或物理参数
3. 更新 `robots/auto_robot.xacro` 中的插件配置
