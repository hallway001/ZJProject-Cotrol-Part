# Loader Control - ROS2控制模块

这是一个基于ROS2的装载车控制模块，支持多种控制算法。

## 快速开始

### 编译项目

```bash
# 方式1: 使用编译脚本（推荐）
cd /home/lzh/ZJProject/控制模块/loader_control
chmod +x compile.sh
./compile.sh          # Release版本
./compile.sh Debug    # Debug版本

# 方式2: 手动编译
cd /home/lzh/ZJProject/控制模块/loader_control
colcon build --packages-select loader_control
source install/setup.bash
```

### 运行程序

```bash
# 启动整个控制栈
ros2 launch loader_control control_stack.launch.py

# 单独运行节点
ros2 run loader_control trajectory_tracker_node
ros2 run loader_control command_modulator_node
```

## 调试方法

### 1. VS Code调试（推荐）

1. 安装C++扩展和CMake Tools扩展
2. 按F5启动调试
3. 选择对应的调试配置（如"调试轨迹跟踪器节点"）
4. 在代码中设置断点

### 2. 命令行调试

```bash
# 编译Debug版本
colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 使用gdb调试
gdb --args ros2 run loader_control trajectory_tracker_node
```

### 3. 日志调试

```bash
# 设置日志级别为DEBUG
ros2 run loader_control trajectory_tracker_node --ros-args --log-level debug
```

详细调试指南请参阅 [BUILD_AND_DEBUG.md](BUILD_AND_DEBUG.md)

## 项目结构

```
loader_control/
├── include/          # 头文件
├── src/             # 源代码
├── launch/          # 启动文件
├── config/          # 配置文件
└── test/            # 测试脚本
```

## 控制器类型

- **PID控制器**: 传统的PID控制算法
- **Pure Pursuit**: 纯跟踪算法
- **MPC**: 模型预测控制（简化版）
- **SMC**: 滑模控制

通过修改 `config/control.yaml` 中的 `controller.type` 参数来切换控制器。

## 依赖项

- ROS2 (Humble/Iron推荐)
- Eigen3
- loader_control_interfaces (需要单独创建消息包)

## 常见问题

1. **编译错误**: 确保所有依赖已安装，运行 `rosdep install --from-paths . --ignore-src -r -y`
2. **运行时错误**: 检查ROS2环境是否已source，使用 `ros2 node list` 查看节点
3. **消息类型错误**: 确保 `loader_control_interfaces` 包已正确创建和编译

更多信息请查看 [BUILD_AND_DEBUG.md](BUILD_AND_DEBUG.md)

