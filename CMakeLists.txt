cmake_minimum_required(VERSION 3.10)
project(mujoco_RGBD)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

# 包含目录
include_directories(${CMAKE_SOURCE_DIR}/mujoco/include)
include_directories(${CMAKE_SOURCE_DIR}/mujoco/include/mujoco)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

# 链接目录
link_directories(${CMAKE_SOURCE_DIR}/mujoco/lib)

# 查找依赖包
find_package(PkgConfig REQUIRED)

# GLFW
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})

# PCL (Point Cloud Library)
find_package(PCL REQUIRED COMPONENTS 
    common 
    io 
    visualization
    filters
    features
    segmentation
    surface
    sample_consensus
    search
    kdtree
    octree
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenGL
find_package(OpenGL REQUIRED)

# 收集源文件
file(GLOB_RECURSE STAIR_PERCEPTION_SOURCES 
    "src/stair_perception/*.cpp"
    "src/stair_perception/*.cc"
)

# 创建楼梯感知库
add_library(stair_perception ${STAIR_PERCEPTION_SOURCES})
target_include_directories(stair_perception PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(stair_perception ${PCL_LIBRARIES})

# 主程序
add_executable(main src/main.cpp)

# 链接库
target_link_libraries(main 
    stair_perception
    mujoco 
    pthread 
    dl 
    m 
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES} 
    ${PCL_LIBRARIES}
)

# 设置运行时库路径
set_target_properties(main PROPERTIES
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/mujoco/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 可选：创建调试版本
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(main PRIVATE DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
endif()

# 安装规则（可选）
install(TARGETS main DESTINATION bin)
install(DIRECTORY model/ DESTINATION share/mujoco_RGBD/model)

# 打印配置信息
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "GLFW libraries: ${GLFW_LIBRARIES}")


