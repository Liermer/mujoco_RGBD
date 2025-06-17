#!/bin/bash

# 运行脚本 - MuJoCo RGBD 项目

set -e

# 检查可执行文件是否存在
if [ ! -f "build/main" ]; then
    echo "错误: 找不到可执行文件 build/main"
    echo "请先运行构建脚本: ./build.sh"
    exit 1
fi

# 进入构建目录
cd build

# 设置库路径
export LD_LIBRARY_PATH="$(pwd)/../mujoco/lib:${LD_LIBRARY_PATH}"

# 检查模型文件是否存在
if [ ! -f "../model/scene.xml" ]; then
    echo "警告: 找不到模型文件 model/scene.xml"
fi

echo "启动 MuJoCo RGBD 演示..."
echo "库路径: $LD_LIBRARY_PATH"
echo ""
echo "控制说明:"
echo "  - 鼠标: 旋转视角"
echo "  - R: 重置物理仿真"
echo "  - Esc: 退出程序"
echo ""

# 运行程序
./main 