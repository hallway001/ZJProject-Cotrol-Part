# 快速开始指南

## 一、编译步骤

### Windows系统

1. **打开命令提示符或PowerShell**

2. **Source ROS2环境**
   ```batch
   call C:\opt\ros\humble\local_setup.bat
   ```

3. **进入项目目录**
   ```batch
   cd "C:\Users\14858\Desktop\ZJProject\控制模块\loader_control"
   ```

4. **编译项目**
   ```batch
   REM 方式1: 使用编译脚本
   ..\compile.bat
   
   REM 方式2: 手动编译
   colcon build --packages-select loader_control
   ```

5. **Source工作空间**
   ```batch
   call install\setup.bat
   ```

### Linux系统

1. **打开终端**

2. **Source ROS2环境**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **进入项目目录**
   ```bash
   cd ~/loader_control_ws  # 或您的项目路径
   ```

4. **编译项目**
   ```bash
   # 方式1: 使用编译脚本
   chmod +x compile.sh
   ./compile.sh
   
   # 方式2: 手动编译
   colcon build --packages-select loader_control
   ```

5. **Source工作空间**
   ```bash
   source install/setup.bash
   ```

## 二、运行程序

### 启动整个控制栈
```bash
ros2 launch loader_control control_stack.launch.py
```

### 单独运行节点
```bash
# 轨迹跟踪器
ros2 run loader_control trajectory_tracker_node

# 指令调制器
ros2 run loader_control command_modulator_node

# 状态估计器
ros2 run loader_control state_estimator_node

# 控制管理器
ros2 run loader_control control_manager_node
```

## 三、调试方法

### 方法1: VS Code调试（最简单）

1. **安装扩展**
   - C/C++ (Microsoft)
   - CMake Tools
   - Python (用于测试脚本)

2. **设置断点**
   - 在代码行号左侧点击设置断点

3. **启动调试**
   - 按 `F5` 或点击调试按钮
   - 选择调试配置（如"调试轨迹跟踪器节点"）

### 方法2: 编译Debug版本

```bash
# 编译Debug版本
colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 使用gdb调试（Linux）
gdb --args ros2 run loader_control trajectory_tracker_node
(gdb) break main
(gdb) run
```

### 方法3: 使用日志

```cpp
// 在代码中添加日志
RCLCPP_DEBUG(this->get_logger(), "变量值: %f", variable);
RCLCPP_INFO(this->get_logger(), "信息");
RCLCPP_WARN(this->get_logger(), "警告");
RCLCPP_ERROR(this->get_logger(), "错误");
```

运行时设置日志级别：
```bash
ros2 run loader_control trajectory_tracker_node --ros-args --log-level debug
```

## 四、常用调试命令

```bash
# 查看运行的节点
ros2 node list

# 查看节点信息
ros2 node info /trajectory_tracker

# 查看话题列表
ros2 topic list

# 监听话题数据
ros2 topic echo /control/raw_command
ros2 topic echo /localization/vehicle_state

# 查看参数
ros2 param list /trajectory_tracker
ros2 param get /trajectory_tracker controller.type

# 设置参数
ros2 param set /trajectory_tracker controller.type pure_pursuit
```

## 五、常见问题

### 1. 编译错误: "找不到loader_control_interfaces"

**解决方案**: 需要先创建并编译消息接口包
```bash
# 创建消息包（如果还没有）
ros2 pkg create --build-type ament_cmake loader_control_interfaces
# 然后定义消息类型并编译
```

### 2. 运行时错误: "节点无法启动"

**检查清单**:
- [ ] ROS2环境已source
- [ ] 工作空间已source (`source install/setup.bash`)
- [ ] 所有依赖已安装
- [ ] 配置文件路径正确

### 3. 调试时断点不生效

**解决方案**:
- 确保编译的是Debug版本
- 检查可执行文件路径是否正确
- 在VS Code中检查调试配置

## 六、下一步

- 查看详细文档: [BUILD_AND_DEBUG.md](BUILD_AND_DEBUG.md)
- 修改配置文件: `config/control.yaml`
- 切换控制器类型: 修改 `controller.type` 参数

