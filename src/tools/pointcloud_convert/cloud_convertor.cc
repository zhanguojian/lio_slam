/**
 * @file cloud_convert.cc
 * @author uanheng (uanheng@foxmail.com)
 * @brief 点云转换
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <glog/logging.h>
#include <iomanip>

#include "common/tools/signal.hpp"
#include "ros_bridge/cloud_convert/cloud_convert.h"

namespace sad {

void CloudConvert::process(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out) {
  switch (lidar_type_) {
    case LidarType::Livox:
      livoxHandler(msg, pcl_out);
      break;
    case LidarType::RoboSense:
      roboSenseHandler(msg, pcl_out);
      break;
    case LidarType::LeiShen:
      leiShenHandler(msg, pcl_out);
      break;
    case LidarType::Custom:
      customHandler(msg, pcl_out);
      break;
    default:
      LOG(ERROR) << "错误的雷达类型，请检查雷达类型配置";
      return;
  }

  pcl_out->width = pcl_out->points.size();
  pcl_out->height = 1;
  pcl_out->is_dense = true;

  return;
}

void CloudConvert::livoxHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out) {
  try {
    // 定义激光点元素迭代器（起始为第一个点）
    PointCloud2ConstIterator<float> iter_x(*msg, "x");
    PointCloud2ConstIterator<float> iter_y(*msg, "y");
    PointCloud2ConstIterator<float> iter_z(*msg, "z");
    PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
    PointCloud2ConstIterator<uint8_t> iter_r(*msg, "line");
    PointCloud2ConstIterator<double> iter_t(*msg, "timestamp");

    pcl_out->clear();
    pcl_out->reserve(msg->width);

    // 获取点云第一个点时间
    uint64_t start = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    // 获取点云最后一个点时间
    PointCloud2ConstIterator<double> end_t(*msg, "timestamp");
    end_t += (static_cast<int32_t>(msg->width) - 1);
    uint64_t end = static_cast<uint64_t>(*end_t);

    // 注意，这里复用了pcl点云类型的stamp和seq，分别用于存储点云起始时间和一帧点云持续时间，单位均为ns
    pcl_out->header.stamp = start;
    pcl_out->header.seq = end - start;

    for (uint32_t point_index = 0; point_index < msg->width; ++point_index) {
      FullPointType added_pt;
      added_pt.x = *iter_x;
      added_pt.y = *iter_y;
      added_pt.z = *iter_z;
      added_pt.intensity = static_cast<uint8_t>(std::clamp(std::round(*iter_i), 0.0f, 65535.0f));
      added_pt.time = (*iter_t - static_cast<double>(start)) * 1e-6;
      added_pt.ring = *iter_r;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
      ++iter_r;
      ++iter_t;

      // 过滤异常点（NAN）
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
        // LOG(INFO) << "xyz nan " << added_pt.x << " " << added_pt.y << " " << added_pt.z;
        continue;
      }

      /// 略掉过近的点
      if (added_pt.getVector3fMap().norm() < point_filter_distance_) {
        continue;
      }

      // 过滤低处点
      if (added_pt.z < point_filter_height_) {
        continue;
      }

      pcl_out->emplace_back(added_pt);
    }

  } catch (const std::exception &e) {
    LOG(ERROR) << "错误的激光点字段，请检查雷达类型配置是否正确";
    error_signal_g.emit(SENOR_ERROR_POINT_TYPE_CRITICAL, "错误的激光点字段，请检查雷达类型配置是否正确");
  }

  return;
}

void CloudConvert::roboSenseHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out) {
  try {
    // 定义激光点字段
    PointCloud2ConstIterator<float> iter_x(*msg, "x");
    PointCloud2ConstIterator<float> iter_y(*msg, "y");
    PointCloud2ConstIterator<float> iter_z(*msg, "z");
    PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
    PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
    PointCloud2ConstIterator<double> iter_t(*msg, "timestamp");

    pcl_out->clear();
    uint32_t cloud_size = msg->data.size() / msg->point_step;
    pcl_out->reserve(cloud_size);

    // 获取点云第一个点时间
    uint64_t start = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    // 获取点云最后一个点时间
    PointCloud2ConstIterator<double> end_t(*msg, "timestamp");
    end_t += (static_cast<int32_t>(cloud_size) - 1);
    uint64_t end = static_cast<uint64_t>(*end_t * 1e9);

    // 注意，这里复用了pcl点云类型的stamp和seq，分别用于存储点云起始时间和一帧点云持续时间，单位均为ns
    pcl_out->header.stamp = start;
    pcl_out->header.seq = end - start;

    double point_start = static_cast<double>(start) * 1e-9;

    for (uint32_t point_index = 0; point_index < cloud_size; ++point_index) {
      FullPointType added_pt;
      added_pt.x = *iter_x;
      added_pt.y = *iter_y;
      added_pt.z = *iter_z;
      added_pt.intensity = static_cast<uint8_t>(std::clamp(std::round(*iter_i), 0.0f, 65535.0f));
      added_pt.time = (*iter_t - point_start) * 1e3;
      added_pt.ring = *iter_r;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
      ++iter_r;
      ++iter_t;

      // 过滤异常点（NAN）
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
        // LOG(INFO) << "xyz nan " << added_pt.x << " " << added_pt.y << " " << added_pt.z;
        continue;
      }

      /// 略掉过近的点
      if (added_pt.getVector3fMap().norm() < point_filter_distance_) {
        continue;
      }

      // 过滤低处点
      if (added_pt.z < point_filter_height_) {
        continue;
      }

      pcl_out->emplace_back(added_pt);
    }

  } catch (const std::exception &e) {
    LOG(ERROR) << "错误的激光点字段，请检查雷达类型配置是否正确";
    error_signal_g.emit(SENOR_ERROR_POINT_TYPE_CRITICAL, "错误的激光点字段，请检查雷达类型配置是否正确");
  }

  return;
}

void CloudConvert::leiShenHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out) {
  try {
    // 定义激光点字段
    PointCloud2ConstIterator<float> iter_x(*msg, "x");
    PointCloud2ConstIterator<float> iter_y(*msg, "y");
    PointCloud2ConstIterator<float> iter_z(*msg, "z");
    PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
    PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
    PointCloud2ConstIterator<double> iter_t(*msg, "time");

    pcl_out->clear();
    pcl_out->reserve(msg->width);

    // 获取点云最后一个点时间
    PointCloud2ConstIterator<double> end_t(*msg, "time");
    end_t += (static_cast<int32_t>(msg->width) - 1);

    // 注意，这里复用了pcl点云类型的stamp和seq，分别用于存储点云起始时间和一帧点云持续时间，单位均为ns
    pcl_out->header.stamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    pcl_out->header.seq = static_cast<uint64_t>(*end_t * 1e9);

    for (uint32_t point_index = 0; point_index < msg->width; ++point_index) {
      FullPointType added_pt;
      added_pt.x = *iter_x;
      added_pt.y = *iter_y;
      added_pt.z = *iter_z;
      added_pt.intensity = static_cast<uint8_t>(std::clamp(std::round(*iter_i), 0.0f, 65535.0f));
      added_pt.time = *iter_t * 1e3;
      added_pt.ring = *iter_r;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
      ++iter_r;
      ++iter_t;

      // 过滤异常点（NAN）
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
        LOG(INFO) << "xyz nan " << added_pt.x << " " << added_pt.y << " " << added_pt.z;
        continue;
      }

      /// 略掉过近的点
      if (added_pt.getVector3fMap().norm() < point_filter_distance_) {
        continue;
      }

      // 过滤低处点
      if (added_pt.z < point_filter_height_) {
        continue;
      }
    }

  } catch (const std::exception &e) {
    LOG(ERROR) << "错误的激光点字段，请检查雷达类型配置是否正确";
    error_signal_g.emit(SENOR_ERROR_POINT_TYPE_CRITICAL, "错误的激光点字段，请检查雷达类型配置是否正确");
  }

  return;
}

void CloudConvert::customHandler(const PointCloud2MsgPtr &msg, FullCloudPtr &pcl_out) {
  PointCloud2ConstIterator<float> iter_x(*msg, "x");
  PointCloud2ConstIterator<float> iter_y(*msg, "y");
  PointCloud2ConstIterator<float> iter_z(*msg, "z");
  PointCloud2ConstIterator<uint16_t> iter_intensity(*msg, "intensity");
  PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
  PointCloud2ConstIterator<double> iter_timestamp(*msg, "timestamp");

  pcl_out->clear();
  pcl_out->reserve(msg->width);

  for (uint32_t point_index = 0; point_index < msg->width; ++point_index) {
    FullPointType added_pt;
    added_pt.x = *iter_x;
    added_pt.y = *iter_y;
    added_pt.z = *iter_z;
    added_pt.intensity = *iter_intensity;
    added_pt.time = *iter_timestamp;
    added_pt.ring = *iter_ring;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_ring;
    ++iter_timestamp;

    // 过滤异常点（NAN）
    if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
      LOG(INFO) << "xyz nan " << added_pt.x << " " << added_pt.y << " " << added_pt.z;
      continue;
    }

    /// 略掉过近的点
    if (added_pt.getVector3fMap().norm() < point_filter_distance_) {
      continue;
    }

    // 过滤低处点
    if (added_pt.z < point_filter_height_) {
      continue;
    }

    pcl_out->emplace_back(std::move(added_pt));
  }

  return;
}

bool CloudConvert::init(const RunConfig &config) {
  point_filter_distance_ = config.point_filter_distance_;
  point_filter_height_ = config.point_filter_height_;

  switch (config.lidar_type_) {
    case 0:
      lidar_type_ = LidarType::Livox;
      LOG(INFO) << "Using Livox Lidar";
      break;
    case 1:
      lidar_type_ = LidarType::RoboSense;
      LOG(INFO) << "Using RoboSense Lidar";
      break;
    case 2:
      lidar_type_ = LidarType::LeiShen;
      LOG(INFO) << "Using LeiShen Lidar";
      break;
    case 3:
      lidar_type_ = LidarType::Custom;
      LOG(INFO) << "Using Custom Lidar";
      break;
    default:
      LOG(ERROR) << "错误的雷达类型，请检查配置";
      error_signal_g.emit(SENOR_ERROR_LIDAR_TYPE_CRITICAL, "错误的雷达类型，请检查配置");
      break;
  }

  return true;
}

bool CloudConvert::init(const int &lidar_type, const double &point_filter_distance, const double &point_filter_height) {
  switch (lidar_type) {
    case 0:
      lidar_type_ = LidarType::Livox;
      break;
    case 1:
      lidar_type_ = LidarType::RoboSense;
      break;
    case 2:
      lidar_type_ = LidarType::LeiShen;
      break;
    case 3:
      lidar_type_ = LidarType::Custom;
      break;
    default:
      LOG(ERROR) << "错误的雷达类型，请检查配置";
      error_signal_g.emit(SENOR_ERROR_LIDAR_TYPE_CRITICAL, "错误的雷达类型，请检查配置");
      break;
  }

  point_filter_distance_ = point_filter_distance;
  point_filter_height_ = point_filter_height;

  return true;
}

}  // namespace sad