# 编译和调试指南

## 一、项目编译

### 1. 准备工作

确保您已经：
- 安装了ROS2（建议使用Humble或Iron版本）
- 安装了colcon构建工具：`pip install colcon-common-extensions`
- 创建了`loader_control_interfaces`消息包（如果还没有）

### 2. 创建工作空间（如果还没有）

```bash
# 在项目父目录下创建工作空间
mkdir -p /home/lzh/loader_control_ws/src
cd /home/lzh/loader_control_ws/src

# 如果loader_control不在src目录下，创建符号链接或复制
# Linux下：
# 假设您的项目在 /home/lzh/ZJProject/控制模块/loader_control
# ln -s /home/lzh/ZJProject/控制模块/loader_control loader_control
# 或者直接复制文件夹到src目录下
# cp -r /home/lzh/ZJProject/控制模块/loader_control .
```

### 3. 编译步骤

```bash
# 1. 进入工作空间根目录
cd /home/lzh/loader_control_ws

# 2. Source ROS2环境（根据您的ROS2版本选择）
# Ubuntu/Linux:
source /opt/ros/humble/setup.bash  # 或 iron, galactic等


# 3. 安装依赖（首次编译时）
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译项目
colcon build --packages-select loader_control

# 5. Source编译后的工作空间
source install/setup.bash
```

### 4. 编译选项

```bash
# 仅编译loader_control包
colcon build --packages-select loader_control

# 编译并显示详细信息
colcon build --packages-select loader_control --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# 编译Debug版本（用于调试）
colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 编译Release版本（优化性能）
colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Release

# 清理编译文件后重新编译
colcon build --packages-select loader_control --cmake-clean-cache
```

## 二、运行程序

### 1. 启动控制栈

```bash
# 确保已经source了工作空间
source install/setup.bash

# 启动整个控制栈
ros2 launch loader_control control_stack.launch.py
```

### 2. 单独运行节点

```bash
# 运行轨迹跟踪器节点
ros2 run loader_control trajectory_tracker_node

# 运行指令调制器节点
ros2 run loader_control command_modulator_node

# 运行状态估计器节点
ros2 run loader_control state_estimator_node

# 运行控制管理器节点
ros2 run loader_control control_manager_node
```

### 3. 检查节点状态

```bash
# 查看所有运行的节点
ros2 node list

# 查看节点信息
ros2 node info /trajectory_tracker

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 监听话题数据
ros2 topic echo /control/raw_command
ros2 topic echo /localization/vehicle_state
```

## 三、调试方法

### 1. 使用VS Code调试（推荐）

#### 配置launch.json

在`.vscode/launch.json`中配置调试任务（见下方配置）

#### 调试步骤：
1. 在代码中设置断点
2. 按F5启动调试
3. 选择对应的调试配置
4. 程序会在断点处暂停

### 2. 使用GDB调试（Linux）

```bash
# 1. 编译Debug版本
colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 2. Source工作空间
source install/setup.bash

# 3. 使用gdb运行节点
gdb --args ros2 run loader_control trajectory_tracker_node

# 4. 在gdb中设置断点
(gdb) break trajectory_tracker_node.cpp:100
(gdb) run

# 5. 常用gdb命令
(gdb) list          # 显示代码
(gdb) next          # 下一行
(gdb) step          # 进入函数
(gdb) print var     # 打印变量
(gdb) continue      # 继续执行
(gdb) quit          # 退出
```

### 3. 使用日志输出调试

```cpp
// 在代码中使用日志
RCLCPP_DEBUG(this->get_logger(), "Debug message");
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error message");
```

设置日志级别：
```bash
# 设置节点日志级别
ros2 run loader_control trajectory_tracker_node --ros-args --log-level debug

# 或在launch文件中设置
<node ...>
  <param name="use_sim_time" value="false"/>
  <env name="RCUTILS_LOGGING_SEVERITY" value="DEBUG"/>
</node>
```

### 4. 使用rqt工具调试

```bash
# 安装rqt工具
sudo apt install ros-humble-rqt*

# 启动rqt
rqt

# 常用插件：
# - rqt_graph: 查看节点和话题关系图
# - rqt_console: 查看日志
# - rqt_topic: 监控话题数据
# - rqt_service_caller: 调用服务
```

## 四、常见问题排查

### 1. 编译错误

```bash
# 检查依赖是否安装
rosdep check --from-paths src --ignore-src

# 清理并重新编译
rm -rf build/ install/ log/
colcon build --packages-select loader_control
```

### 2. 运行时错误

```bash
# 查看节点日志
ros2 node info /node_name

# 查看所有日志
ros2 topic echo /rosout

# 检查消息类型是否匹配
ros2 interface show loader_control_interfaces/msg/VehicleState
```

### 3. 参数问题

```bash
# 列出节点参数
ros2 param list /trajectory_tracker

# 获取参数值
ros2 param get /trajectory_tracker controller.type

# 设置参数
ros2 param set /trajectory_tracker controller.type pure_pursuit
```

## 五、性能分析

```bash
# 使用perf工具（Linux）
perf record -g ros2 run loader_control trajectory_tracker_node
perf report

# 使用valgrind检查内存泄漏
valgrind --leak-check=full ros2 run loader_control trajectory_tracker_node
```

## 六、单元测试

```bash
# 运行测试
colcon test --packages-select loader_control

# 查看测试结果
colcon test-result --verbose
```

