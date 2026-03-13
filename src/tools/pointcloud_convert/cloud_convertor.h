/**
 * @file cloud_convert.h
 * @author uanheng (uanheng@foxmail.com)
 * @brief 点云转换
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/config_def.h"
#include "common/types/point_types.h"
#include "ros_bridge/message_def.h"

namespace slam_tools {

/**
 * @brief 通用点云消息转换函数
 *
 */
class CloudConvert {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class LidarType {
    Livox = 0,  ///< 览沃
    RoboSense,  ///< 速腾
    LeiShen,    ///< 镭神
    Custom,     ///< 自定义（X、Y、Z、I、R、T）
  };

  CloudConvert() = default;
  ~CloudConvert() = default;

  /**
   * @brief处理sensor_msgs::PointCloud2点云
   *
   * @param msg
   * @param pcl_out
   */
  void process(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out);

  /**
   * @brief 初始化函数，直接读取配置结构体
   *
   * @param config
   * @return true
   * @return false
   */
  bool init(const RunConfig &config);

  /**
   * @brief 初始化函数，直接读取配置结构体

   * @param lidar_type
   * @param point_filter_distance
   * @param point_filter_height
   * @return true
   * @return false
   */
  bool init(const int &lidar_type, const double &point_filter_distance, const double &point_filter_height);

 private:
  /**
   * @brief mid360雷达处理函数
   *
   * @param msg
   */
  void livoxHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out);

  /**
   * @brief mid360雷达处理函数
   *
   * @param msg
   */
  void roboSenseHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out);

  /**
   * @brief 镭神雷达处理函数
   *
   * @param msg
   */
  void leiShenHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out);

  /**
   * @brief 镭神雷达处理函数
   *
   * @param msg
   */
  void customHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out);

  /// @brief 雷达类型
  LidarType lidar_type_ = LidarType::Custom;

  /// @brief 过滤点最小距离
  double point_filter_distance_ = 2;

  /// @brief 过滤点最小高度
  double point_filter_height_ = 0;
};
}  // namespace slam_tools