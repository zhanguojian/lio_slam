add_library(${PROJECT_NAME}.ch4
        gins_pre_integ.cc
        imu_preintegration.cc
        g2o_types.cc
        )

# ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}.ch4
        rclcpp
        sensor_msgs
)

ADD_EXECUTABLE(test_preintegration test_preintegration.cc)
ADD_TEST(test_preintegration test_preintegration)

target_link_libraries(test_preintegration
        gtest pthread glog::glog gflags ${PROJECT_NAME}.ch4 ${PROJECT_NAME}.ch3 ${PROJECT_NAME}.common
        )

add_executable(run_gins_pre_integ run_gins_pre_integ.cc)
target_link_libraries(run_gins_pre_integ
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch4
        ${g2o_libs}
        )
