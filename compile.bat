@echo off
REM ROS2 控制模块编译脚本 (Windows)

setlocal enabledelayedexpansion

echo === ROS2 Loader Control 编译脚本 (Windows) ===

REM 检查ROS2环境
if "%ROS_DISTRO%"=="" (
    echo 警告: ROS2环境未设置，尝试设置...
    if exist "C:\opt\ros\humble\local_setup.bat" (
        call "C:\opt\ros\humble\local_setup.bat"
        echo 已source ROS2 Humble环境
    ) else if exist "C:\opt\ros\iron\local_setup.bat" (
        call "C:\opt\ros\iron\local_setup.bat"
        echo 已source ROS2 Iron环境
    ) else (
        echo 错误: 未找到ROS2环境，请手动source
        exit /b 1
    )
)

REM 获取脚本所在目录
set SCRIPT_DIR=%~dp0
cd /d "%SCRIPT_DIR%"

REM 检查是否为工作空间结构
if not exist "src" (
    echo 当前目录不是colcon工作空间，使用当前目录作为包目录
    set PACKAGE_DIR=.
) else (
    echo 检测到colcon工作空间结构
    set PACKAGE_DIR=src
    cd /d "%SCRIPT_DIR%\.."
)

REM 检查loader_control包是否存在
if not exist "loader_control\package.xml" (
    if not exist "%PACKAGE_DIR%\loader_control\package.xml" (
        echo 错误: 未找到loader_control包
        exit /b 1
    )
)

REM 编译选项
set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

echo 开始编译 (构建类型: %BUILD_TYPE%)...

if "%BUILD_TYPE%"=="Debug" (
    colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
) else (
    colcon build --packages-select loader_control --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
)

REM 检查编译结果
if %ERRORLEVEL% EQU 0 (
    echo 编译成功！
    echo 请运行以下命令source工作空间:
    echo call install\setup.bat
    
    set /p SOURCE_NOW="是否现在source工作空间? (y/n) "
    if /i "!SOURCE_NOW!"=="y" (
        call install\setup.bat
        echo 已source工作空间
    )
) else (
    echo 编译失败！请检查错误信息
    exit /b 1
)

endlocal

