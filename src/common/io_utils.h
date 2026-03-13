//
// Created by xiang on 2021/7/20.
// Modified for ROS2 Humble
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <fstream>
#include <functional>
#include <utility>
#include <memory>

#include "common/dataset_type.h"
#include "common/global_flags.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/lidar_utils.h"
#include "common/math_utils.h"
#include "common/message_def.h"
#include "common/odom.h"
#include "common/point_types.h"

#include "ch3/utm_convert.h"


namespace sad {
/**
 * 读取本书提供的数据文本文件，并调用回调函数
 * 数据文本文件主要提供IMU/Odom/GNSS读数
 */
class TxtIO {
   public:
    TxtIO(const std::string &file_path) : fin(file_path) {}

    /// 定义回调函数
    using IMUProcessFuncType = std::function<void(const IMU &)>;
    using OdomProcessFuncType = std::function<void(const Odom &)>;
    using GNSSProcessFuncType = std::function<void(const GNSS &)>;

    TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }

    TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }

    // 遍历文件内容，调用回调函数
    void Go();

   private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;
};

/**
 * ROSBAG2 IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 * ROS2 版本使用 rosbag2_cpp
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::LIVOX)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        assert(dataset_type_ != DatasetType::UNKNOWN);
    }

    /// 消息处理回调类型
    using PointCloud2Handle = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;

    using GNSSHandle = std::function<bool(GNSSPtr)>;
    using OdomHandle = std::function<bool(const Odom &)>;

    using Scan2DHandle = std::function<bool(Scan2d::Ptr)>;
    // 遍历文件内容，调用回调函数
    void Go();

    /// 根据数据集类型自动确定topic名称
    RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f);

    /// 根据数据集自动处理RTK消息
    RosbagIO &AddAutoRTKHandle(GNSSHandle f);

    /// point cloud 2 的处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        pc2_handles_.emplace(topic_name, f);
        return *this;
    }

    /// IMU
    RosbagIO &AddImuHandle(ImuHandle f);
    
    /// IMU with specific topic name
    RosbagIO &AddImuHandle(const std::string& topic_name, ImuHandle f) {
        imu_handles_.emplace(topic_name, f);
        return *this;
    }

    //scan 2d
    RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
    scan2d_handles_.emplace(topic_name, f);
    return *this;
}
    
    /// 清除现有的处理函数
    void CleanProcessFunc() {
        scan2d_handles_.clear();
        pc2_handles_.clear();
        imu_handles_.clear();
        gnss_handles_.clear();
    }

   private:
    /// 根据设定的数据集名称获取雷达名
    std::string GetLidarTopicName() const;

    /// 根据数据集名称确定IMU topic名称
    std::string GetIMUTopicName() const;

    /// 反序列化消息
    template <typename MessageT>
    std::shared_ptr<MessageT> DeserializeMessage(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message) {
        rclcpp::Serialization<MessageT> serialization;
        auto msg = std::make_shared<MessageT>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&serialized_msg, msg.get());
        return msg;
    }

    std::map<std::string, Scan2DHandle> scan2d_handles_;
    std::map<std::string, PointCloud2Handle> pc2_handles_;
    std::map<std::string, ImuHandle> imu_handles_;
    std::map<std::string, GNSSHandle> gnss_handles_;

    std::string bag_file_;
    DatasetType dataset_type_;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IO_UTILS_H
