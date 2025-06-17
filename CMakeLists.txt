cmake_minimum_required(VERSION 3.10)

project(mujoco_camera)

# Compiler settings
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# finding modules
find_package(OpenCV REQUIRED) 
find_package(PCL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(X11 REQUIRED)

# MuJoCo configuration
set(MUJOCO_ROOT "/home/ziyi/.mujoco/mujoco-3.3.0")
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_ROOT}/include")
set(MUJOCO_LIBRARY_DIRS "${MUJOCO_ROOT}/lib")
set(MUJOCO_LIBRARIES "${MUJOCO_ROOT}/lib/libmujoco.so")

include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${MUJOCO_INCLUDE_DIRS})

                    
link_directories(${PCL_LIBRARY_DIRS}
                 ${MUJOCO_LIBRARY_DIRS})

# add_executable(camera_test src/camera.cpp)                

# target_link_libraries(camera_test 
#             ${MUJOCO_LIBRARIES}
#             ${OpenCV_LIBS}
#             ${PCL_LIBRARIES})     

add_executable(multi_win_test src/two_window.cpp
                                src/mujoco_rgbd_camera.cpp)     

target_link_libraries(multi_win_test
            ${MUJOCO_LIBRARIES}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            glfw
            X11)              