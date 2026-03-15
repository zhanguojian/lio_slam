//
// Modified by guojian on 2026/3/15.
// 
//
#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/point_types.h"

/// 雷达扫描的一些消息定义和工具函数 (ROS2版本)
using Scan2d = sensor_msgs::msg::LaserScan;
using Scan2dPtr = sensor_msgs::msg::LaserScan::SharedPtr;
using MultiScan2d = sensor_msgs::msg::MultiEchoLaserScan;

namespace sad {

// 将MultiEchoLaserScan转换为LaserScan
inline Scan2d::SharedPtr MultiToScan2d(MultiScan2d::SharedPtr mscan) {
    auto scan = std::make_shared<Scan2d>();
    scan->header = mscan->header;
    scan->range_max = mscan->range_max;
    scan->range_min = mscan->range_min;
    scan->angle_increment = mscan->angle_increment;
    scan->angle_max = mscan->angle_max;
    scan->angle_min = mscan->angle_min;
    for (auto r : mscan->ranges) {
        if (r.echoes.empty()) {
            scan->ranges.emplace_back(scan->range_max + 1.0);
        } else {
            scan->ranges.emplace_back(r.echoes[0]);
        }
    }
    for (auto i : mscan->intensities) {
        if (i.echoes.empty()) {
            scan->intensities.emplace_back(0);
        } else {
            scan->intensities.emplace_back(i.echoes[0]);
        }
    }
    scan->scan_time = mscan->scan_time;
    scan->time_increment = mscan->time_increment;

    // limit range max
    scan->range_max = 20.0;
    return scan;
}

/// ROS2 PointCloud2 转通常的pcl PointCloud
inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

/**
 * 其他类型点云转到PointType点云
 * 用的最多的是全量点云转到XYZI点云
 * @tparam PointT
 * @param input
 * @return
 */
template <typename PointT = FullPointType>
CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
    CloudPtr cloud(new PointCloudType);
    for (auto& pt : input->points) {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        cloud->points.template emplace_back(p);
    }
    cloud->width = input->width;
    return cloud;
}

/// 对点云进行voxel filter,指定分辨率
inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
    return value.array().template round().template cast<int>();
}

}  // namespace sad

