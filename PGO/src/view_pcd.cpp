#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <algorithm>

namespace fs = boost::filesystem;

class PCDPublisher {
public:
    PCDPublisher(ros::NodeHandle& nh, const std::string& dir, float voxel_size, double rate_hz)
        : nh_(nh), voxel_size_(voxel_size), rate_hz_(rate_hz) {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_points", 1, true);
        loadPCDFiles(dir);
    }

    void publishIncremental() {
        ros::Rate rate(rate_hz_);

        for (size_t idx = 0; idx < clouds_.size(); ++idx) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(clouds_[idx]);
            vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            vg.filter(*cloud_down);

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloud_down, msg);
            msg.header.frame_id = "map";   // ✅ 所有点云统一到世界系
            msg.header.stamp = ros::Time::now();
            pub_.publish(msg);

            ROS_INFO("Published session_%zu with %zu points", idx, cloud_down->points.size());

            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("All sessions published.");
    }

private:
    void loadPCDFiles(const std::string& dir) {
        if (!fs::exists(dir) || !fs::is_directory(dir)) {
            ROS_ERROR("Directory does not exist: %s", dir.c_str());
            return;
        }
        for (auto& entry : fs::directory_iterator(dir)) {
            if (fs::is_regular_file(entry) && entry.path().extension() == ".pcd") {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                if (pcl::io::loadPCDFile(entry.path().string(), *cloud) == -1) {
                    ROS_WARN("Could not read file %s", entry.path().string().c_str());
                    continue;
                }
                clouds_.push_back(cloud);
                file_names_.push_back(entry.path().string());
            }
        }

        // 按文件名排序 (session_0.pcd, session_1.pcd, …)
        std::sort(file_names_.begin(), file_names_.end());
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sorted;
        for (auto& name : file_names_) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(name, *cloud);
            sorted.push_back(cloud);
        }
        clouds_ = sorted;

        ROS_INFO("Loaded %zu PCD files", clouds_.size());
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    float voxel_size_;
    double rate_hz_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
    std::vector<std::string> file_names_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh("~");

    std::string dir;
    float voxel_size;
    double rate_hz;
    nh.param<std::string>("pcd_dir", dir, "/tmp/pcd");
    nh.param<float>("voxel_size", voxel_size, 0.1f);  // 默认 10cm
    nh.param<double>("rate_hz", rate_hz, 0.5);        // 每 2 秒发一个

    PCDPublisher publisher(nh, dir, voxel_size, rate_hz);
    publisher.publishIncremental();

    ros::spin();
    return 0;
}
