#!/bin/bash

# 构建脚本 - MuJoCo RGBD 项目
# 使用方法: ./build.sh [clean|debug|release]

set -e  # 遇到错误时退出

# 设置默认构建类型
BUILD_TYPE="Release"
CLEAN_BUILD=false

# 解析命令行参数
case "$1" in
    "clean")
        CLEAN_BUILD=true
        echo "执行清理构建..."
        ;;
    "debug")
        BUILD_TYPE="Debug"
        echo "构建调试版本..."
        ;;
    "release")
        BUILD_TYPE="Release"
        echo "构建发布版本..."
        ;;
    "")
        echo "使用默认构建类型: Release"
        ;;
    *)
        echo "用法: $0 [clean|debug|release]"
        exit 1
        ;;
esac

# 检查必要的依赖
echo "检查依赖..."
command -v cmake >/dev/null 2>&1 || { echo "错误: 需要安装 cmake"; exit 1; }
command -v make >/dev/null 2>&1 || { echo "错误: 需要安装 make"; exit 1; }

# 检查 MuJoCo 库是否存在
if [ ! -f "mujoco/lib/libmujoco.so" ]; then
    echo "警告: 找不到 MuJoCo 库文件 mujoco/lib/libmujoco.so"
    echo "请确保已正确安装 MuJoCo 并放置在 mujoco/ 目录下"
fi

# 创建/清理构建目录
if [ "$CLEAN_BUILD" = true ] && [ -d "build" ]; then
    echo "清理旧的构建目录..."
    rm -rf build
fi

mkdir -p build
cd build

# 设置库路径
export LD_LIBRARY_PATH="$(pwd)/../mujoco/lib:${LD_LIBRARY_PATH}"

# 配置 CMake
echo "配置 CMake..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..

# 编译
echo "开始编译..."
make -j$(nproc)

# 编译完成
echo ""
echo "编译完成！"
echo "可执行文件位置: $(pwd)/main"
echo ""
echo "运行方法:"
echo "  cd build"
echo "  export LD_LIBRARY_PATH=\"\$(pwd)/../mujoco/lib:\${LD_LIBRARY_PATH}\""
echo "  ./main"
echo ""
echo "或者直接运行:"
echo "  ./run.sh" 