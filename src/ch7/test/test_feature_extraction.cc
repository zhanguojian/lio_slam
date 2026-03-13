//
// Created by xiang on 2022/7/18.
// Modified for ROS2 Humble
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/loam-like/feature_extraction.h"
#include "common/io_utils.h"

#include "common/timer/timer.h"
#include "common/point_cloud_utils.h"
#include "common/lidar_utils.h"

/// ROS2 版本使用已解码的点云话题
DEFINE_string(bag_path, "./dataset/sad/wxb/test1.bag", "path to wxb bag");
DEFINE_string(topic, "/velodyne_points", "point cloud topic");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 测试角点和平面点的提取
    sad::FeatureExtraction feature_extraction;

    system("rm -rf ./data/ch7/*.pcd");

    sad::RosbagIO bag_io(fLS::FLAGS_bag_path);
    bag_io
        .AddPointCloud2Handle(FLAGS_topic,
                              [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) -> bool {
                                  // 转换为 FullCloud
                                  sad::FullCloudPtr cloud(new sad::FullPointCloudType);
                                  pcl::fromROSMsg(*msg, *cloud);
                                  
                                  sad::CloudPtr pcd_corner(new sad::PointCloudType), pcd_surf(new sad::PointCloudType);
                                  sad::common::Timer::Evaluate(
                                      [&]() { feature_extraction.Extract(cloud, pcd_corner, pcd_surf); },
                                      "Feature Extraction");
                                  LOG(INFO) << "original pts:" << cloud->size() << ", corners: " << pcd_corner->size()
                                            << ", surf: " << pcd_surf->size();
                                  sad::SaveCloudToFile("./data/ch7/corner.pcd", *pcd_corner);
                                  sad::SaveCloudToFile("./data/ch7/surf.pcd", *pcd_surf);
                                  return true;
                              })
        .Go();

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
