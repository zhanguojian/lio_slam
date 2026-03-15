# Copyright(c) 2019 livoxtech limited.
cmake_minimum_required(VERSION 3.0)

# we only need livox message definition here
project(livox_ros_driver)

# 这个目录是旧的 ROS1 catkin 包；当前仓库的 ROS2 代码并不依赖它。
# 如果它被上层工程误加入构建，而本机又没有 ROS1/catkin，就直接跳过，
# 避免阻塞主工程的配置过程。
find_package(catkin QUIET COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_generation
        )

if(NOT catkin_FOUND)
    message(STATUS "Skipping thirdparty/livox_ros_driver because catkin was not found.")
    return()
endif()

add_message_files(FILES
        CustomPoint.msg
        CustomMsg.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        )

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif ()