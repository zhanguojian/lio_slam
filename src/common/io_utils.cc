//
// Modified by guojian on 2026/3/15.
//
//



#include "common/io_utils.h"
#include "ch3/utm_convert.h"
#include "dataset_type.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sad {

void TxtIO::Go() {
    if (!fin) {
        LOG(ERROR) << "未能找到文件";
        return;
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            continue;
        }

        if (line[0] == '#') {
            // 以#开头的是注释
            continue;
        }

        // load data from line
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;

        if (data_type == "IMU" && imu_proc_) {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
        } else if (data_type == "ODOM" && odom_proc_) {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            odom_proc_(Odom(time, wl, wr));
        } else if (data_type == "GNSS" && gnss_proc_) {
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            gnss_proc_(GNSS(time, 4, Vec3d(lat, lon, alt), heading, heading_valid));
        }
    }

    LOG(INFO) << "done.";
}

std::string RosbagIO::GetLidarTopicName() const {
    if (dataset_type_ == DatasetType::NCLT) {
        return nclt_lidar_topic;
    }
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_lidar_topic;
    }
    if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_lidar_topic;
    }
    if (dataset_type_ == DatasetType::UTBM) {
        return utbm_lidar_topic;
    }
    if (dataset_type_ == DatasetType::AVIA) {
        return avia_lidar_topic;
    }
    if (dataset_type_ == DatasetType::LIVOX) {
        return livox_lidar_topic;
    }
    return "";
}

void RosbagIO::Go() {
    LOG(INFO) << "running in " << bag_file_;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_;
    storage_options.storage_id = "sqlite3";  // ROS2 默认格式

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        LOG(ERROR) << "cannot open " << bag_file_ << ": " << e.what();
        return;
    }

    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        const std::string& topic = bag_message->topic_name;

        // 处理 PointCloud2
        auto pc2_it = pc2_handles_.find(topic);
        if (pc2_it != pc2_handles_.end()) {
            auto msg = DeserializeMessage<sensor_msgs::msg::PointCloud2>(bag_message);
            pc2_it->second(msg);
            continue;
        }

        // 处理 IMU
        auto imu_it = imu_handles_.find(topic);
        if (imu_it != imu_handles_.end()) {
            auto msg = DeserializeMessage<sensor_msgs::msg::Imu>(bag_message);
            IMUPtr imu;
            double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            
            imu = std::make_shared<IMU>(
                timestamp,
                Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
        
            imu_it->second(imu);
            continue;
        }

        // 处理 GNSS
        auto gnss_it = gnss_handles_.find(topic);
        if (gnss_it != gnss_handles_.end()) {
            auto msg = DeserializeMessage<sensor_msgs::msg::NavSatFix>(bag_message);
            double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            GNSSPtr gnss = std::make_shared<GNSS>(
                timestamp, 
                static_cast<int>(msg->status.status),
                Vec3d(msg->latitude, msg->longitude, msg->altitude),
                0.0, false);
            ConvertGps2UTMOnlyTrans(*gnss);
            if (!std::isnan(gnss->lat_lon_alt_[2])) {
                gnss_it->second(gnss);
            }
            continue;
        }

        if (global::FLAG_EXIT) {
            break;
        }
    }

    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

RosbagIO &RosbagIO::AddAutoPointCloudHandle(PointCloud2Handle f) {
    if (dataset_type_ == DatasetType::AVIA) {
        // AVIA 需要特殊处理 Livox 消息格式
        LOG(WARNING) << "AVIA dataset requires Livox message handling - not yet implemented for ROS2";
        return *this;
    } else if (dataset_type_ == DatasetType::WXB_3D) {
        // WXB 需要 Velodyne packets 解析
        LOG(WARNING) << "WXB_3D dataset requires Velodyne packets handling - not yet implemented for ROS2";
        return *this;
    } else if (dataset_type_ == DatasetType::LIVOX) {
        // WXB 需要 Velodyne packets 解析
        LOG(WARNING) << "WXB_3D dataset requires Velodyne packets handling - not yet implemented for ROS2";
        return *this;        
    } else {
        return AddPointCloud2Handle(GetLidarTopicName(), f);
    }
}

RosbagIO &RosbagIO::AddAutoRTKHandle(GNSSHandle f) {
    if (dataset_type_ == DatasetType::NCLT) {
        gnss_handles_.emplace(nclt_rtk_topic, f);
    } else {
        LOG(WARNING) << "RTK handling for this dataset type is not yet implemented";
    }
    return *this;
}

RosbagIO &RosbagIO::AddImuHandle(ImuHandle f) {
    imu_handles_.emplace(GetIMUTopicName(), f);
    return *this;
}

std::string RosbagIO::GetIMUTopicName() const {
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_imu_topic;
    } else if (dataset_type_ == DatasetType::UTBM) {
        return utbm_imu_topic;
    } else if (dataset_type_ == DatasetType::NCLT) {
        return nclt_imu_topic;
    } else if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_imu_topic;
    } else if (dataset_type_ == DatasetType::AVIA) {
        return avia_imu_topic;
    } else if (dataset_type_ == DatasetType::LIVOX) {
        return livox_imu_topic;
    } else {
        LOG(ERROR) << "cannot load imu topic name of dataset " << int(dataset_type_);
    }
    return "";
}
}  // namespace sad
