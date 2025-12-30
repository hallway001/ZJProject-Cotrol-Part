#!/bin/bash

# ROS2 控制模块编译脚本

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 Loader Control 编译脚本 ===${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告: ROS2环境未设置，尝试source...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}已source ROS2 Humble环境${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}已source ROS2 Iron环境${NC}"
    else
        echo -e "${RED}错误: 未找到ROS2环境，请手动source${NC}"
        exit 1
    fi
fi

# 获取脚本所在目录（假设脚本在loader_control目录下）
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 检查是否为工作空间结构
if [ ! -d "src" ]; then
    echo -e "${YELLOW}当前目录不是colcon工作空间，使用当前目录作为包目录${NC}"
    PACKAGE_DIR="."
else
    echo -e "${GREEN}检测到colcon工作空间结构${NC}"
    PACKAGE_DIR="src"
    cd "$SCRIPT_DIR/.."  # 回到工作空间根目录
fi

# 检查loader_control包是否存在
if [ ! -f "loader_control/package.xml" ] && [ ! -f "$PACKAGE_DIR/loader_control/package.xml" ]; then
    echo -e "${RED}错误: 未找到loader_control包${NC}"
    exit 1
fi

# 安装依赖
echo -e "${GREEN}检查并安装依赖...${NC}"
if command -v rosdep &> /dev/null; then
    rosdep update
    rosdep install --from-paths . --ignore-src -r -y || echo -e "${YELLOW}某些依赖可能无法通过rosdep安装${NC}"
else
    echo -e "${YELLOW}警告: rosdep未安装，跳过依赖检查${NC}"
fi

# 编译选项
BUILD_TYPE=${1:-Release}  # 默认为Release，可以传入Debug

echo -e "${GREEN}开始编译 (构建类型: $BUILD_TYPE)...${NC}"

if [ "$BUILD_TYPE" == "Debug" ]; then
    colcon build --packages-select loader_control \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug \
        --symlink-install
else
    colcon build --packages-select loader_control \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --symlink-install
fi

# 检查编译结果
if [ $? -eq 0 ]; then
    echo -e "${GREEN}编译成功！${NC}"
    echo -e "${GREEN}请运行以下命令source工作空间:${NC}"
    echo -e "${YELLOW}source install/setup.bash${NC}"
    
    # 询问是否自动source
    read -p "是否现在source工作空间? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        source install/setup.bash
        echo -e "${GREEN}已source工作空间${NC}"
    fi
else
    echo -e "${RED}编译失败！请检查错误信息${NC}"
    exit 1
fi

