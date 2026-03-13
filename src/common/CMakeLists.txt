# ROS2 common library

add_library(${PROJECT_NAME}.common
        io_utils.cc
        timer/timer.cc
        global_flags.cc
        g2o_types.cc
        point_cloud_utils.cc
        ${PROJECT_SOURCE_DIR}/src/ch3/utm_convert.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        ${PCL_LIBRARIES}
        )

# ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}.common
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_conversions
        rosbag2_cpp
)

