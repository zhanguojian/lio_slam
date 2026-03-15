# 自定义查找模块
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2 / ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# 基础依赖
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gflags REQUIRED)
find_package(CSparse REQUIRED)
find_package(Cholmod REQUIRED)

# thirdparty 头文件
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/velodyne/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o)

# 公共头文件搜索路径
if(TARGET Eigen3::Eigen)
    get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
endif()

include_directories(SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

# 兼容不同 Pangolin 导出方式
if(NOT TARGET Pangolin::pangolin)
    add_library(Pangolin::pangolin INTERFACE IMPORTED)
    set_target_properties(Pangolin::pangolin PROPERTIES
        INTERFACE_LINK_LIBRARIES "${Pangolin_LIBRARIES}"
    )
endif()

# 内置 g2o：关闭 apps/examples，加快构建并减少无关依赖
set(G2O_BUILD_APPS OFF CACHE BOOL "Build g2o apps" FORCE)
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "Build g2o examples" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/g2o)

# 项目实际使用到的 g2o 组件
set(g2o_libs
    core
    stuff
    solver_eigen
    solver_dense
    solver_cholmod
    types_sba
)

if(BUILD_WITH_UBUNTU1804)
    function(extract_file filename extract_dir)
        message(STATUS "Extract ${filename} to ${extract_dir} ...")
        set(temp_dir ${extract_dir})
        if(EXISTS ${temp_dir})
            file(REMOVE_RECURSE ${temp_dir})
        endif()
        file(MAKE_DIRECTORY ${temp_dir})
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${filename}
            WORKING_DIRECTORY ${temp_dir})
    endfunction()

    set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
    set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

    extract_file(${PROJECT_SOURCE_DIR}/thirdparty/tbb/2019_U8.tar.gz ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8)
    include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

    tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
        compiler=gcc-9
        stdver=c++17
        ${TBB_BUILD_DIR}
        ${TBB_BUILD_PREFIX}
        CONFIG_DIR
        TBB_DIR)

    find_package(TBB REQUIRED)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
    link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)
    set(_tbb_lib TBB::tbb)
else()
    set(_tbb_lib tbb)
endif()

set(third_party_libs
    ${g2o_libs}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Pangolin::pangolin
    glog::glog
    gflags
    yaml-cpp
    ${yaml-cpp_LIBRARIES}
    ${CHOLMOD_LIBRARY}
    ${CSPARSE_LIBRARY}
    ${_tbb_lib}
)