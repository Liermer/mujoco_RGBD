cmake_minimum_required(VERSION 3.10)
project(mujoco_cpp_gui)

include_directories(${CMAKE_SOURCE_DIR}/mujoco/include)
include_directories(${CMAKE_SOURCE_DIR}/mujoco/include/mujoco)

link_directories(${CMAKE_SOURCE_DIR}/mujoco/lib)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})


include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(main src/main.cpp)
target_link_libraries(main mujoco pthread dl m GL ${GLFW_LIBRARIES})


