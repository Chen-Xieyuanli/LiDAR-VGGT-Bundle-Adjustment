#include "tools.hpp"
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <ctime>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <malloc.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


ros::Publisher pcd_pub;
ros::Publisher pose_pub;
ros::Publisher merge_body_pcd_pub;
ros::Publisher chunk_first_poses_pub;

// ----------- 读取 TUM pose -----------
int read_TUM_pose(std::string filename, std::vector<IMUST> &x_buf) {
    std::fstream file(filename);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    while (file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        Eigen::Quaterniond qd(qw, qx, qy, qz);
        Eigen::Vector3d td(tx, ty, tz);
        IMUST curr;
        curr.R = qd.toRotationMatrix();
        curr.p = td;
        curr.t = timestamp;
        x_buf.push_back(curr);
    }
    return x_buf.size();
}

// ----------- 读取点云 -----------
void read_point_clouds(const std::string &base_path, int num,
                       std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &point_clouds)
{
    std::cout << "Loading " << num << " pcd files from " << base_path << std::endl;
    for (int i = 0; i < num; ++i)
    {
        std::string filename = base_path + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile(filename, *cloud) < 0) {
            std::cerr << "Could not load file: " << filename << std::endl;
            continue;
        }
        std::cout << filename << " loaded, points: " << cloud->size() << std::endl;
        if (!cloud->empty()) point_clouds.push_back(cloud);
    }
}

// ----------- 下采样函数 -----------
pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float leaf_size = 0.05f)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*filtered);
    return filtered;
}

// ----------- 转换 chunk 到第一帧 body 系 -----------
void transform_chunk_to_body(const std::vector<IMUST> &chunk_poses,
                             const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &chunk_points,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &body_cloud)
{
    body_cloud->clear();
    if (chunk_poses.empty() || chunk_points.empty()) return;

    const IMUST &first_pose = chunk_poses[0];
    const Eigen::Matrix3d &R0 = first_pose.R;
    const Eigen::Vector3d &p0 = first_pose.p;

    for (size_t i = 0; i < chunk_points.size(); ++i)
    {
        const IMUST &pose_i = chunk_poses[i];
        const Eigen::Matrix3d &Ri = pose_i.R;
        const Eigen::Vector3d &pi = pose_i.p;

        for (const auto &pt : *chunk_points[i])
        {
            Eigen::Vector3d Pi(pt.x, pt.y, pt.z);
            Eigen::Vector3d Pw = Ri * Pi + pi;
            Eigen::Vector3d Pb0 = R0.transpose() * (Pw - p0);

            pcl::PointXYZI new_pt = pt;
            new_pt.x = Pb0.x();
            new_pt.y = Pb0.y();
            new_pt.z = Pb0.z();
            new_pt.intensity = 1.0;
            body_cloud->push_back(new_pt);
        }
    }
}

// ----------- 保存 pose -----------
void save_TUM_pose(const std::string &filename, const std::vector<IMUST> &x_buf) {
    std::ofstream file(filename);
    for (const auto &pose : x_buf) {
        Eigen::Quaterniond qd(pose.R);
        file << pose.t << " "
             << pose.p.x() << " " << pose.p.y() << " " << pose.p.z() << " "
             << qd.x() << " " << qd.y() << " " << qd.z() << " " << qd.w() << "\n";
    }
    file.close();
}

// ----------- 保存点云 -----------
void save_chunk_point_clouds(const std::string &base_path,
                                 int chunk_id,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "Warning: chunk " << chunk_id << " cloud is empty, not saving." << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_xyz->reserve(cloud->size());

    for (const auto &pt : *cloud) {
        pcl::PointXYZ p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        cloud_xyz->push_back(p);
    }

    std::string filename = base_path + "chunk" + std::to_string(chunk_id) + ".pcd";
    pcl::io::savePCDFileASCII(filename, *cloud_xyz);
    std::cout << "Saved chunk " << chunk_id << " with " << cloud_xyz->size() << " points (XYZ only) to " << filename << std::endl;
}


// ----------- 可视化 pose + 点云 -----------
void visualize_chunk_pose_pcd(const std::vector<IMUST> &chunk_poses,
                              const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &body_clouds)
{
    if (chunk_poses.empty() || body_clouds.empty()) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZI>());

    for (size_t i = 0; i < chunk_poses.size(); ++i)
    {
        const IMUST &pose = chunk_poses[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "chunk_pose";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pose.p.x();
        marker.pose.position.y = pose.p.y();
        marker.pose.position.z = pose.p.z();

        Eigen::Quaterniond q(pose.R);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        pose_pub.publish(marker);

        for (const auto &pt : *body_clouds[i]) {
            Eigen::Vector3d Pb(pt.x, pt.y, pt.z);
            Eigen::Vector3d Pw = pose.R * Pb + pose.p;
            pcl::PointXYZI new_pt = pt;
            new_pt.x = Pw.x();
            new_pt.y = Pw.y();
            new_pt.z = Pw.z();
            cloud_total->push_back(new_pt);
        }
    }

    cloud_total = voxel_downsample(cloud_total, 0.5f);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_total, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();

    ros::Rate r(10);
    for (int k = 0; k < 30; k++) {
        pcd_pub.publish(cloud_msg);
        ros::spinOnce();
        r.sleep();
    }
}

// ----------- 可视化 merged 点云 -----------
void visualize_merged_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr &merged_cloud)
{
    if (!merged_cloud || merged_cloud->empty()) return;

    auto filtered = voxel_downsample(merged_cloud, 0.25f);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();

    ros::Rate r(10);
    for (int k = 0; k < 30; k++) {
        merge_body_pcd_pub.publish(cloud_msg);
        ros::spinOnce();
        r.sleep();
    }
}

// ----------- 可视化每个 chunk 第一帧位姿 -----------
void visualize_chunk_first_poses(const std::vector<IMUST> &chunk_first_poses)
{
    if (chunk_first_poses.empty()) return;

    for (size_t i = 0; i < chunk_first_poses.size(); ++i)
    {
        const IMUST &pose = chunk_first_poses[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "chunk_first_pose";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pose.p.x();
        marker.pose.position.y = pose.p.y();
        marker.pose.position.z = pose.p.z();

        Eigen::Quaterniond q(pose.R);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        ros::Rate r(10);
        for (int k = 0; k < 30; k++) {
            marker.header.stamp = ros::Time::now();
            chunk_first_poses_pub.publish(marker);
            ros::spinOnce();
            r.sleep();
        }
    }
}

// ----------- 主函数 -----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_to_chunk");
    ros::NodeHandle nh;

    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("chunk_pcd", 10);
    pose_pub = nh.advertise<visualization_msgs::Marker>("chunk_pose", 10);
    merge_body_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("merge_body_pcd", 10);
    chunk_first_poses_pub = nh.advertise<visualization_msgs::Marker>("chunk_first_poses", 10);

    int chunk_num;
    std::string data_base_path;
    std::string result_path;
    double visualize_time;

    nh.getParam("chunk_num", chunk_num);
    nh.getParam("data_base_path", data_base_path);
    nh.getParam("result_path", result_path);
    nh.getParam("visualize_time", visualize_time);

    std::vector<IMUST> chunk_first_poses;

    for (int i = 0; i < chunk_num; ++i)
    {
        std::vector<IMUST> chunk_poses;
        int chunk_size = read_TUM_pose(data_base_path + "chunk" + std::to_string(i) + "/camera/pose.txt", chunk_poses);
        chunk_first_poses.push_back(chunk_poses[0]);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> point_clouds;
        read_point_clouds(data_base_path + "chunk" + std::to_string(i) + "/camera/pcd/", chunk_size, point_clouds);

        std::cout << "Visualizing pose and point cloud in chunk " << i << std::endl;
        // visualize_chunk_pose_pcd(chunk_poses, point_clouds);

        pcl::PointCloud<pcl::PointXYZI>::Ptr body_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        transform_chunk_to_body(chunk_poses, point_clouds, body_cloud);

        std::cout << "Visualizing merged point cloud in body frame" << std::endl;
        // visualize_merged_pcd(body_cloud);

        save_chunk_point_clouds(result_path, i, body_cloud);
    }

    // visualize_chunk_first_poses(chunk_first_poses);
    save_TUM_pose(result_path + "chunk_first_poses.txt", chunk_first_poses);
    std::cout << "saving downed point clouds and first poses done." << std::endl;

    ros::spin();
    return 0;
}