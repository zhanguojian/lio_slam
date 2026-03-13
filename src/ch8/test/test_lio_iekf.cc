//
// Created by xiang on 22-11-10.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch8/lio-iekf/lio_iekf.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "./dataset/sad/nclt/20120115.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    sad::LioIEKF lio;
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) -> bool {
            sad::common::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "IEKF lio");
            return true;
        })
        // NOTE: AddLivoxHandle 暂未适配 ROS2
        .AddImuHandle([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    lio.Finish();
    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
