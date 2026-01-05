# 仿真使用指南

本指南介绍如何使用仿真节点来测试控制系统，无需实际的传感器数据。

## 概述

仿真节点 (`simulation_node`) 会生成以下传感器数据并发布到相应的话题：

- `/loader/imu` - IMU 数据（姿态、角速度、加速度）
- `/loader/scan` - LiDAR 扫描数据（模拟）
- `/loader/wheel_speed` - 轮速数据
- `/loader/steering_rate` - 转向角速度
- `/loader/steering/feedback` - 转向反馈角度
- `/loader/planning/path` - 参考路径

## 使用方法

### 方法 1: 仅启动仿真节点

```bash
ros2 launch loader_control simulation.launch.py
```

可选参数：
- `path_type`: 路径类型 (`circle` 或 `straight`)，默认 `circle`
- `path_radius`: 圆形路径半径（米），默认 `10.0`
- `vehicle_speed`: 车辆速度（m/s），默认 `1.0`
- `publish_rate`: 发布频率（Hz），默认 `50.0`

示例：
```bash
# 圆形路径，半径 15 米，速度 2 m/s
ros2 launch loader_control simulation.launch.py path_type:=circle path_radius:=15.0 vehicle_speed:=2.0

# 直线路径
ros2 launch loader_control simulation.launch.py path_type:=straight vehicle_speed:=1.5
```

### 方法 2: 同时启动仿真和控制节点

```bash
ros2 launch loader_control control_with_simulation.launch.py
```

这会同时启动：
- 仿真节点（生成传感器数据）
- 控制节点（执行路径跟踪控制）

可选参数（包括仿真和控制参数）：
```bash
ros2 launch loader_control control_with_simulation.launch.py \
    path_type:=circle \
    path_radius:=10.0 \
    vehicle_speed:=1.0 \
    controller_type:=pure_pursuit \
    use_actuator_models:=true
```

### 方法 3: 分别启动（用于调试）

**终端 1 - 启动仿真：**
```bash
ros2 run loader_control simulation_node
```

**终端 2 - 启动控制：**
```bash
ros2 launch loader_control control.launch.py
```

## 仿真参数说明

### 路径类型

1. **圆形路径 (circle)**
   - 车辆沿着圆形路径行驶
   - 需要设置 `path_radius` 参数
   - 适用于测试转向控制性能

2. **直线路径 (straight)**
   - 车辆沿着直线行驶
   - 适用于测试纵向控制性能

### 车辆动力学模型

仿真节点使用简化的车辆动力学模型：
- 圆形路径：车辆位置按 `x = r*cos(θ)`, `y = r*sin(θ)` 计算
- 角速度：`ω = v / r`（圆形路径）
- 转向反馈：基于路径曲率计算 `δ = atan(L * κ)`

其中：
- `r`: 路径半径
- `v`: 车辆速度
- `L`: 轴距（假设 2.5m）
- `κ`: 路径曲率

## 可视化

可以使用 RViz 可视化仿真结果：

```bash
rviz2
```

添加以下显示项：
- **Path**: `/loader/planning/path` (类型: Path)
- **Marker**: `/loader/control/visualization` (类型: Marker)
- **TF**: 查看坐标系变换

## 调试技巧

1. **查看话题列表：**
   ```bash
   ros2 topic list
   ```

2. **查看传感器数据：**
   ```bash
   ros2 topic echo /loader/imu
   ros2 topic echo /loader/wheel_speed
   ros2 topic echo /loader/planning/path
   ```

3. **查看控制命令：**
   ```bash
   ros2 topic echo /loader/cmd_vel
   ```

4. **查看节点状态：**
   ```bash
   ros2 node list
   ros2 node info /simulation_node
   ros2 node info /loader_control_node
   ```

## 注意事项

1. **时间同步**：仿真节点以固定频率发布数据，确保时间戳正确
2. **坐标系**：所有数据使用 `map` 或 `base_link` 坐标系
3. **参数匹配**：确保控制节点的参数与仿真设置匹配
4. **性能**：仿真节点以 50Hz 频率运行，可根据需要调整

## 故障排除

### 问题：控制节点没有接收到数据

**解决方案：**
1. 检查话题是否正确发布：
   ```bash
   ros2 topic list | grep loader
   ```
2. 检查数据接收器的参数配置
3. 确保时间同步设置正确

### 问题：路径跟踪误差较大

**可能原因：**
1. 控制器参数需要调整
2. 仿真速度与控制器期望不匹配
3. 路径曲率过大

**解决方案：**
1. 调整控制器增益参数
2. 降低仿真速度
3. 增加路径半径

## 扩展仿真

可以修改 `src/simulation_node.cpp` 来：
- 添加更复杂的路径（8字形、S形等）
- 添加噪声和延迟模拟
- 添加障碍物检测
- 实现更精确的车辆动力学模型

