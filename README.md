# 装载车无人驾驶控制模块 (Loader Autonomous Driving Control Module)

## 项目简介

本项目实现了装载车无人驾驶控制模块，支持通过ROS2接收多源传感器数据（IMU、激光雷达、轮速、角速度）和规划路径，并采用PID、Pure Pursuit等控制器进行路径跟踪，生成油门和转向控制指令。

### 核心特性

- ✅ **多源传感器数据接收与同步**：支持IMU、激光雷达、轮速、角速度传感器数据的时间同步接收
- ✅ **路径跟踪控制器**：实现了PID和Pure Pursuit两种控制器，支持动态切换
- ✅ **执行器物理建模**：显式建模执行延迟、死区、比例偏差、噪声等物理特性
- ✅ **转向误差监控**：实时监控转向角误差，支持可量化的误差分析与日志记录
- ✅ **高可维护性配置**：采用分层YAML配置文件管理，支持车型/工况切换
- ✅ **ROS2集成**：完全基于ROS2框架，支持参数动态重载

---

## 项目结构

```
loader_control/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS2包配置
├── README.md                   # 本文档
│
├── config/                     # 配置文件目录
│   ├── common/                 # 通用配置
│   │   ├── sensors.yaml       # 传感器配置
│   │   └── actuator.yaml      # 执行器物理模型参数
│   ├── controllers/            # 控制器配置
│   │   ├── pid.yaml           # PID控制器参数
│   │   ├── pure_pursuit.yaml  # Pure Pursuit控制器参数
│   │   └── mpc.yaml           # MPC控制器参数（预留）
│   ├── loader_v1/             # 小型装载车配置
│   │   ├── actuator.yaml      # 执行器参数覆盖
│   │   └── steering_limits.yaml
│   ├── loader_v2/             # 大型装载车配置
│   │   └── actuator.yaml
│   └── error_monitor.yaml     # 误差监控配置
│
├── include/loader_control/    # 头文件目录
│   ├── vehicle_state.hpp      # 车辆状态结构
│   ├── data_receiver.hpp      # 数据接收与同步模块
│   ├── actuator_model.hpp     # 执行器物理模型
│   ├── controller_base.hpp    # 控制器基类
│   ├── pid_controller.hpp     # PID控制器
│   ├── pure_pursuit_controller.hpp  # Pure Pursuit控制器
│   ├── steering_error_monitor.hpp   # 转向误差监控
│   └── control_node.hpp       # 主控制节点
│
├── src/                        # 源文件目录
│   ├── main.cpp               # 程序入口
│   ├── data_receiver.cpp      # 数据接收实现
│   ├── actuator_model.cpp     # 执行器模型实现
│   ├── controller_base.cpp    # 控制器基类实现
│   ├── pid_controller.cpp     # PID控制器实现
│   ├── pure_pursuit_controller.cpp  # Pure Pursuit实现
│   ├── steering_error_monitor.cpp   # 误差监控实现
│   └── control_node.cpp       # 主节点实现
│
├── launch/                     # Launch文件
│   └── control.launch.py      # 主启动文件
│
└── scripts/                    # Python工具脚本
    ├── param_merge.py         # 配置文件合并工具
    └── error_analyzer.py      # 误差数据分析工具
```

---

## 快速开始

### 1. 依赖安装

```bash
# ROS2 (推荐Humble或Iron版本)
# 确保已安装ROS2基础包

# 安装Python依赖（用于工具脚本）
pip3 install pyyaml pandas matplotlib numpy
```

### 2. 编译项目

```bash
# 在工作空间根目录
cd ~/your_workspace
colcon build --packages-select loader_control
source install/setup.bash
```

### 3. 运行控制节点

```bash
# 使用默认配置（PID控制器，loader_v1车型）
ros2 launch loader_control control.launch.py

# 指定控制器类型
ros2 launch loader_control control.launch.py controller_type:=pure_pursuit

# 指定车型
ros2 launch loader_control control.launch.py vehicle_model:=loader_v2 controller_type:=pid
```

### 4. 参数配置

所有参数通过YAML配置文件管理，支持运行时动态调整：

```bash
# 查看当前参数
ros2 param list /loader_control_node

# 动态修改参数
ros2 param set /loader_control_node controller_type pure_pursuit
ros2 param set /loader_control_node pid_controller.lateral.kp 2.5
```

---

## 配置说明

### 执行器物理模型参数 (`config/common/actuator.yaml`)

```yaml
actuator:
  steering:
    delay_time_constant: 0.15      # 一阶延迟时间常数 τ (s)
    dead_zone: 0.02                 # 死区宽度 (±rad)
    scale_bias: 1.02                # 比例偏差
    noise_std: 0.005                # 噪声标准差 (rad)
    max_angle: 0.785                # 最大转向角 (rad)
    filter_alpha: 0.8               # 低通滤波系数
```

### PID控制器参数 (`config/controllers/pid.yaml`)

```yaml
pid_controller:
  longitudinal:
    kp: 1.5
    ki: 0.1
    kd: 0.3
  lateral:
    kp: 2.0
    ki: 0.05
    kd: 0.5
  steering_compensation:
    enable: true
    ff_gain: 0.3                    # 转向误差前馈补偿增益
```

### Pure Pursuit控制器参数 (`config/controllers/pure_pursuit.yaml`)

```yaml
pure_pursuit_controller:
  lookahead:
    base_distance: 2.0              # 基础前瞻距离 (m)
    velocity_gain: 0.5              # 速度增益
  error_compensation:
    enable: true
    lookahead_error_gain: 0.5       # 基于误差的前瞻调整增益
```

---

## 使用说明

### 数据接口

控制节点订阅以下Topic：

- `/loader/imu` - IMU数据 (`sensor_msgs/Imu`)
- `/loader/scan` - 激光雷达数据 (`sensor_msgs/LaserScan`)
- `/loader/wheel_speed` - 轮速数据 (`std_msgs/Float64`, m/s)
- `/loader/steering_rate` - 转向角速度 (`std_msgs/Float64`, rad/s)
- `/loader/steering/feedback` - 转向角反馈 (`std_msgs/Float64`, rad)
- `/loader/planning/path` - 规划路径 (`nav_msgs/Path`)

控制节点发布以下Topic：

- `/loader/cmd_vel` - 控制指令 (`geometry_msgs/Twist`)
  - `linear.x`: 油门指令 (m/s)
  - `angular.z`: 转向角指令 (rad)
- `/loader/control/steering_error` - 实时转向误差 (`std_msgs/Float64`)
- `/loader/control/visualization` - 可视化标记 (`visualization_msgs/Marker`)

### 误差监控

转向误差监控模块自动记录误差数据到CSV文件（`logs/steering_error_YYYYMMDD_HHMMSS.csv`），包含：

- `timestamp`: 时间戳
- `delta_cmd`: 转向指令 (rad)
- `delta_fb`: 转向反馈 (rad)
- `error`: 误差 = delta_cmd - delta_fb (rad)
- `velocity_x`: 速度 (m/s)
- `curvature`: 路径曲率 (1/m)
- `rmse`: 滑动窗口RMSE

使用分析工具处理日志：

```bash
# 分析误差日志
python3 scripts/error_analyzer.py logs/steering_error_20250105_120000.csv -o results/ -p

# 生成报告和图表
python3 scripts/error_analyzer.py logs/steering_error_*.csv --output-dir results/ --plot --report
```

### 配置文件合并工具

```bash
# 合并多个配置文件
python3 scripts/param_merge.py \
  -b config/common/actuator.yaml \
  -o config/loader_v1/actuator.yaml \
  -o config/controllers/pid.yaml \
  --output merged_config.yaml
```

---

## 控制器对比实验

项目设计支持多控制器对照实验，可用于评估：

1. **路径跟踪精度**：横向偏差、航向偏差RMSE
2. **控制平滑性**：控制量变化率
3. **初始偏差敏感性**：不同初始侧偏下的收敛时间
4. **转向误差影响**：存在执行误差时的性能表现

实验流程：

```bash
# 1. 运行PID控制器，记录数据
ros2 launch loader_control control.launch.py controller_type:=pid
# 运行测试场景，数据自动记录到 logs/

# 2. 运行Pure Pursuit控制器
ros2 launch loader_control control.launch.py controller_type:=pure_pursuit

# 3. 分析对比结果
python3 scripts/error_analyzer.py logs/*.csv -o results/ -p
```

---

## 开发与扩展

### 添加新控制器

1. 继承 `ControllerBase` 基类
2. 实现 `computeCommand()` 方法
3. 在 `control_node.cpp` 的 `initializeController()` 中注册
4. 创建对应的配置文件（`config/controllers/your_controller.yaml`）

### 扩展传感器支持

在 `data_receiver.hpp/cpp` 中添加新的订阅者，并在 `VehicleState` 中扩展状态字段。

### ROS2 Control集成

项目预留了ROS2 Control集成接口，可在 `control_node.cpp` 中实现 `hardware_interface` 插件，实现与底层执行器的直接连接。

---

## 技术文档

详细技术文档请参考：

- [控制器接口设计说明](docs/controller_interface.md) - 待补充
- [执行器建模指南](docs/actuator_modeling.md) - 待补充
- [配置文件管理规范](docs/config_management.md) - 待补充

---

## 许可证

Apache-2.0 License

---

## 作者

lhw

---

## 版本历史

- **v1.0.0** (2026-01-05)
  - 初始版本
  - 实现PID和Pure Pursuit控制器
  - 支持执行器物理建模
  - 实现转向误差监控

---

## 常见问题

**Q: 如何调整控制器参数？**  
A: 修改 `config/controllers/` 下对应的YAML文件，或使用 `ros2 param set` 动态调整。

**Q: 误差日志保存在哪里？**  
A: 默认保存在 `logs/` 目录，可在 `config/error_monitor.yaml` 中配置。

**Q: 如何禁用执行器模型？**  
A: 启动时设置 `use_actuator_models:=false`，或在配置文件中设置。

---

## 联系方式

如有问题或建议，请提交Issue或联系项目维护者。

