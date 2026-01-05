# 故障排除指南

## 共享库找不到的问题

### 错误信息
```
error while loading shared libraries: libloader_control.so: cannot open shared object file: No such file or directory
```

### 问题原因

这个错误通常由以下原因引起：

1. **环境变量未设置**：没有 source `install/setup.bash`，导致 `LD_LIBRARY_PATH` 没有包含库文件路径
2. **RPATH 未正确设置**：可执行文件的 RPATH 没有指向库文件位置
3. **库文件未正确安装**：编译时库文件没有被正确安装到 install 目录

### 解决方案

#### 方法 1: 确保 source 环境（最重要！）

**每次打开新终端后，必须执行：**
```bash
cd ~/Control_Module
source install/setup.bash
```

或者添加到 `~/.bashrc` 中：
```bash
echo "source ~/Control_Module/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 方法 2: 重新编译并检查安装

```bash
cd ~/Control_Module

# 清理旧的编译文件
rm -rf build install log

# 重新编译
colcon build --packages-select loader_control

# Source 环境
source install/setup.bash

# 验证库文件是否存在
ls -la install/loader_control/lib/loader_control/

# 应该看到：
# - libloader_control.so (共享库)
# - loader_control_node (可执行文件)
```

#### 方法 3: 手动设置 LD_LIBRARY_PATH（临时方案）

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Control_Module/install/loader_control/lib/loader_control
```

#### 方法 4: 检查库文件依赖

```bash
# 检查可执行文件的依赖
ldd install/loader_control/lib/loader_control/loader_control_node

# 应该看到 libloader_control.so 的路径
```

### 验证修复

运行以下命令验证问题是否解决：

```bash
# 1. 检查环境变量
echo $LD_LIBRARY_PATH | grep loader_control

# 2. 检查库文件
ls install/loader_control/lib/loader_control/libloader_control.so

# 3. 尝试运行节点
ros2 run loader_control loader_control_node --help
```

### 常见问题

#### Q: 为什么每次都要 source？

A: ROS 2 使用工作空间（workspace）的概念，每个工作空间的 `install/setup.bash` 设置了：
- `LD_LIBRARY_PATH`：动态库搜索路径
- `PATH`：可执行文件搜索路径
- `ROS_PACKAGE_PATH`：ROS 包搜索路径
- 其他环境变量

#### Q: 可以永久设置吗？

A: 可以，将 `source` 命令添加到 `~/.bashrc`：
```bash
echo "source ~/Control_Module/install/setup.bash" >> ~/.bashrc
```

#### Q: 为什么 CMakeLists.txt 中设置了 RPATH 还是找不到？

A: RPATH 只在编译时设置，但 ROS 2 的 colcon 构建系统会管理这些路径。最重要的是确保：
1. 库和可执行文件都正确安装
2. 环境变量正确设置（通过 source setup.bash）

### 调试步骤

如果问题仍然存在，按以下步骤调试：

```bash
# 1. 检查库文件是否存在
find install -name "libloader_control.so"

# 2. 检查可执行文件
find install -name "loader_control_node"

# 3. 检查可执行文件的依赖
ldd install/loader_control/lib/loader_control/loader_control_node

# 4. 检查环境变量
env | grep -E "LD_LIBRARY_PATH|ROS"

# 5. 检查 ROS 2 环境
printenv | grep ROS
```

### 相关文件

- `CMakeLists.txt`：构建配置，包含库和可执行文件的安装设置
- `install/setup.bash`：环境设置脚本，必须 source 才能运行节点

