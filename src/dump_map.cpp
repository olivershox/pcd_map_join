
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map>
#include "keyframe.h"
#include "eigen_types.h"
#include "point_types.h"
DEFINE_double(voxel_size,0.1,"地图分辨率");
DEFINE_string(pose_source,"lidar","使用的POSE");
DEFINE_string(dump_to,"./data","导出地图路径");
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// 定义系统中用到的点和点云类型
using namespace std;
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;
using IdType = unsigned long;
/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    cloud->swap(*output);
}
/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}
int main(int argc ,char ** argv){
    google::ParseCommandLineFlags(&argc,&argv,true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    std::map<IdType,KFPtr> keyframes;
    if (!LoadKeyFrames("./data/keyframes.txt", keyframes)) {
        LOG(ERROR) << "failed to load keyframes.txt";
        return -1;
    }

    if (keyframes.empty()) {
        LOG(INFO) << "keyframes are empty";
        return 0;
    }
    LOG(INFO) << "merging";
    CloudPtr global_cloud(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = FLAGS_voxel_size;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    int cnt = 0;
    for (auto& kfp : keyframes) {
        auto kf = kfp.second;
        SE3 pose;
        if (FLAGS_pose_source == "rtk") {
            pose = kf->rtk_pose_;
        } else if (FLAGS_pose_source == "lidar") {
            pose = kf->lidar_pose_;
        } else if (FLAGS_pose_source == "opti1") {
            pose = kf->opti_pose_1_;
        } else if (FLAGS_pose_source == "opti2") {
            pose = kf->opti_pose_2_;
        }

        kf->LoadScan("./data/");

        CloudPtr cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix());

        // voxel size
        CloudPtr kf_cloud_voxeled(new PointCloudType);
        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*kf_cloud_voxeled);

        *global_cloud += *kf_cloud_voxeled;
        kf->cloud_ = nullptr;

        LOG(INFO) << "merging " << cnt << " in " << keyframes.size() << ", pts: " << kf_cloud_voxeled->size()
                  << " global pts: " << global_cloud->size();
        cnt++;
    }

    if (!global_cloud->empty()) {
        SaveCloudToFile(FLAGS_dump_to + "/map.pcd", *global_cloud);
    }

    LOG(INFO) << "done.";
    return 0;
}

   
    


    


