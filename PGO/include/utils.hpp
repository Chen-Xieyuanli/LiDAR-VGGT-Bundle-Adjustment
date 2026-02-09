#include <ros/ros.h>
#include "hba/mypcl.hpp"
#include "balm2/bavoxel.hpp"
#include "balm2/tools.hpp"

#include <gtsam/geometry/Pose3.h> // GTSAM 位姿表示
#include <gtsam/slam/PriorFactor.h> // GTSAM 先验因子
#include <gtsam/slam/BetweenFactor.h> // GTSAM 两帧之间的因子
#include <gtsam/nonlinear/Values.h> // GTSAM 非线性方程组的结果
#include <gtsam/nonlinear/ISAM2.h> // GTSAM 增量式优化

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>  // Include for TF broadcasting
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <malloc.h>
#include <queue>
#include <cmath>

void assign_qt(Eigen::Quaterniond& q, Eigen::Vector3d& t,
               const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_)
{
  q.w() = q_.w(); q.x() = q_.x(); q.y() = q_.y(); q.z() = q_.z();
  t(0) = t_(0); t(1) = t_(1); t(2) = t_(2);
}
//pose with drone_id
struct pose
{
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
            Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0), int id = 0):q(_q), t(_t), pose_status(id){}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    int pose_status; //用以区分优化的pose来自于回环后ba的第几次，第一次loop_ba 0, 之后扩散1，2，3.......................
};
struct pos{
    double x;
    double y;
    double z;
};
//用作可视化回环周围的球
struct sphere{
    Eigen::Vector3d center;  //球心
    double radius;      //半径
    double alpha;     //透明度
};
//读取pose
std::vector<pose> read_pose(std::string filename, int id,
                              Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                              Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
{
    std::vector<pose> pose_vec;
    std::fstream file;
    file.open(filename);
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        double tx, ty, tz, w, x, y, z;
        if (iss >> tx >> ty >> tz >> w >> x >> y >> z)
        {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Vector3d t(tx, ty, tz);
            pose_vec.push_back(pose(qe * q, qe * t + te, id));
        }
    }

    file.close();
    return pose_vec;
}
//读取pose
std::vector<pose> read_pose_with_idx(std::string filename,
                              Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                              Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
{
    std::vector<pose> pose_vec;
    std::fstream file;
    file.open(filename);
    double tx, ty, tz, w, x, y, z;
    int status;
    while (true)
    {
        file >> tx >> ty >> tz >> w >> x >> y >> z >> status;
        Eigen::Quaterniond q(w, x, y, z);
        Eigen::Vector3d t(tx, ty, tz);
        pose_vec.push_back(pose(qe * q, qe * t + te, status));
        if (file.eof()) break;  // 先读取数据,再检查是否到达文件结尾

    }


    file.close();
    return pose_vec;
}
//读取pose.json
int read_json_pose(std::vector<pose> &pose_vec, std::string filename, int id)
{
    filename = filename + "pose.json";
    std::fstream file;
    file.open(filename);
    pose_vec = read_pose(filename, id);
    int pose_size = pose_vec.size();
    file.close();
    std::cout << "[Debug] pose_vec loaded! => Size: " << pose_size << std::endl;
    return pose_size;
}
//
int read_json_pose_new(std::vector<pose> &pose_vec, std::string filename, int id, std::string name)
{
    filename = filename + name + ".json";
    std::fstream file;
    file.open(filename);
    pose_vec = read_pose(filename, id);
    int pose_size = pose_vec.size();
    file.close();
    std::cout << "[Debug] pose_vec loaded! => Size: " << pose_size << std::endl;
    return pose_size;
}
//读取pose.json，带有进行BALM的次数
int read_json_pose_with_idx(std::vector<pose> &pose_vec, std::string filename, int id, int json_num)
{
    filename = filename + "pose_" + std::to_string(json_num) + ".json";
    std::fstream file;
    file.open(filename);
    pose_vec = read_pose_with_idx(filename);
    int pose_size = pose_vec.size();
    file.close();
    std::cout << "[Debug] pose_vec loaded! => Size: " << pose_size << std::endl;
    return pose_size;
}
//写pose
void write_pose_with_idx(std::vector<pose>& pose_vec, std::string path, int i)
{
    // 将文件后缀拼接到文件名中
    std::string filename;
    if(i==-1){
    filename = path + "pose_0.json";
    }else{
    filename = path + "pose_" + std::to_string(i) + ".json";
    }

    // 先以 trunc 模式打开文件，清空内容
    std::ofstream file;
    file.open(filename, std::ofstream::trunc);
    file.close();

    // 获取初始位姿的四元数和位移
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));

    // 以 append 模式打开文件，写入新的位姿信息
    file.open(filename, std::ofstream::app);

    for (size_t i = 0; i < pose_vec.size(); i++)
    {
        // 将姿态相对于第一个姿态进行变换
        // pose_vec[i].t << q0.inverse() * (pose_vec[i].t - t0);
        // pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
        // pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
        // pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
        // pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();

        // 写入位移和四元数
        file << pose_vec[i].t(0) << " "
            << pose_vec[i].t(1) << " "
            << pose_vec[i].t(2) << " "
            << pose_vec[i].q.w() << " " << pose_vec[i].q.x() << " "
            << pose_vec[i].q.y() << " " << pose_vec[i].q.z() << " " << pose_vec[i].pose_status;

        // 如果不是最后一个姿态，添加换行符
        if (i < pose_vec.size() - 1) 
            file << "\n";
    }

    // 关闭文件
    file.close();
}
//读取多机器人回环数据
void read_multi_loop(const string &data_path, std::vector<std::pair<int, int>> &end_idx, std::vector<std::pair<int, int>> &start_idx, std::vector<gtsam::Pose3> &T_start_end, std::vector<double> &overlap, std::vector<double> &score){
    // 打开文件
    std::ifstream file(data_path);
    // 如果文件打开，输出错误并返回
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << data_path << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        int temp_end_idx, temp_start_idx;
        int end_drone_id, start_drone_id;
        double temp_overlap, temp_score;
        std::vector<double> matrix_data(16);
        // 读取 start_idx
        stream >> start_drone_id >> temp_start_idx; 
        std::pair<int, int> start_pair(start_drone_id, temp_start_idx);
        start_idx.push_back(start_pair);
        stream >> end_drone_id >> temp_end_idx;
        std::pair<int, int> end_pair(end_drone_id, temp_end_idx);
        end_idx.push_back(end_pair);
        // 读取 4x4 变换矩阵
        for (int i = 0; i < 16; ++i) {
            stream >> matrix_data[i];
        }
        // 将变换矩阵数据转换为 gtsam::Pose3
        gtsam::Rot3 rotation(
            matrix_data[0], matrix_data[1], matrix_data[2],
            matrix_data[4], matrix_data[5], matrix_data[6],
            matrix_data[8], matrix_data[9], matrix_data[10]
        );
        gtsam::Point3 translation(matrix_data[3], matrix_data[7], matrix_data[11]);
        T_start_end.push_back(gtsam::Pose3(rotation, translation));
        // 读取 overlap 和 score
        stream >> temp_overlap >> temp_score;
        overlap.push_back(temp_overlap);
        score.push_back(temp_score);
    }
    // 关闭文件
    file.close();
}
// Function to create a 4x4 transformation matrix from translation and quaternion
Eigen::Matrix4d createTransformationMatrix(const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;
    return transformation;
}
//根据回环生成回环附近的球
void generate_loop_sphere(std::vector<sphere> &sphere_vecs, std::vector<pose> *pose_vecs, std::vector<std::pair<int, int>> end_idx, std::vector<std::pair<int, int>> start_idx, std::vector<gtsam::Pose3> T_start_end, std::vector<double> overlap, std::vector<double> score, double normal_radius){
    for (int i = 0; i < end_idx.size(); i++)
    {
        sphere temp_sphere;
        temp_sphere.center = (pose_vecs[end_idx[i].first][end_idx[i].second].t + pose_vecs[start_idx[i].first][start_idx[i].second].t) / 2;
        temp_sphere.radius = normal_radius; // + overlap[i] * 0.5 + score[i] * 0.5;     /**************** search radius at loop point*****************/
        temp_sphere.alpha = 0.5;  //1.0 - overlap[i] * 0.5 - score[i] * 0.5;
        sphere_vecs.push_back(temp_sphere);
    }
}
//可视化函数
void visualizeIMUandPointCloud(
    const std::vector<IMUST>& poses_vec_ori,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& pcl_vec_ori,
    ros::Publisher& pose_pub,
    ros::Publisher& pcl_pub,
    const std_msgs::ColorRGBA& color, double t, double voxel_size)
{
    ros::NodeHandle nh;

    // 创建合并后的点云（直接在世界坐标系）
    pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>);

    // 直接转换所有点云到世界坐标系
    for (size_t i = 0; i < poses_vec_ori.size(); ++i) {
        // 获取当前帧位姿（假设已经是世界坐标系）
        const IMUST& pose = poses_vec_ori[i];
        
        // 构造世界坐标系的变换矩阵
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translate(pose.p);
        transform.rotate(pose.R);

        // 转换点云到世界坐标系
        pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*pcl_vec_ori[i], *transformed_cloud, transform.matrix().cast<float>());

        // 应用降采样
        if (voxel_size > 0.0) {
            pcl::VoxelGrid<PointType> voxel_filter;
            voxel_filter.setInputCloud(transformed_cloud);
            voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
            voxel_filter.filter(*transformed_cloud);
        }
        
        // 合并点云
        *combined_cloud += *transformed_cloud;
    }

    // 合并后再次降采样（可选）
    if (voxel_size > 0.0 && !combined_cloud->empty()) {
        pcl::VoxelGrid<PointType> final_filter;
        final_filter.setInputCloud(combined_cloud);
        final_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        final_filter.filter(*combined_cloud);
    }

    ros::Rate loop_rate(10);
    double duration = t;
    double start_time = ros::Time::now().toSec();

    while (ros::ok() && (ros::Time::now().toSec() - start_time) < duration) {
        // 创建位姿标记（直接使用世界坐标）
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < poses_vec_ori.size(); ++i) {
            const IMUST& pose = poses_vec_ori[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "imu_poses";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;

            // 直接使用世界坐标位置
            marker.pose.position.x = pose.p.x();
            marker.pose.position.y = pose.p.y();
            marker.pose.position.z = pose.p.z();

            // 直接使用世界坐标系旋转
            Eigen::Quaterniond quat(pose.R);
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            marker.pose.orientation.w = quat.w();

            // 设置比例和颜色
            marker.scale.x = 0.1;  // 箭头长度
            marker.scale.y = 0.05; // 箭头宽度
            marker.scale.z = 0.05; // 箭头高度
            marker.color = color;

            marker_array.markers.push_back(marker);
        }

        // 创建点云消息
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*combined_cloud, pcl_msg);
        pcl_msg.header.frame_id = "map";
        pcl_msg.header.stamp = ros::Time::now();

        // 发布数据
        pose_pub.publish(marker_array);
        pcl_pub.publish(pcl_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
// void visualizeIMUandPointCloud(
//     const std::vector<IMUST>& poses_vec_ori,
//     const std::vector<pcl::PointCloud<PointType>::Ptr>& pcl_vec_ori,
//     ros::Publisher& pose_pub,
//     ros::Publisher& pcl_pub,
//     const std_msgs::ColorRGBA& color, double t, double voxel_size)
// {
//      ros::NodeHandle nh;

//     // 获取第一帧的位姿作为参考坐标系（世界系）
//     IMUST first_pose = poses_vec_ori[0];
//     Eigen::Matrix3d R0_inv = first_pose.R.transpose();
//     Eigen::Vector3d p0 = first_pose.p;

//     // 创建合并后的点云
//     pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>);

//     // 转换所有点云到世界坐标系
//     for (size_t i = 0; i < poses_vec_ori.size(); ++i) {
//         // 获取当前帧位姿
//         const IMUST& pose = poses_vec_ori[i];
        
//         // 计算相对第一帧的变换：T_world_current = T_world_first * T_first_current
//         Eigen::Matrix3d R = R0_inv * pose.R;
//         Eigen::Vector3d t = R0_inv * (pose.p - p0);

//         // 构造完整的仿射变换矩阵
//         Eigen::Affine3d transform = Eigen::Affine3d::Identity();
//         transform.translate(t);
//         transform.rotate(R);

//         // 转换点云
//         pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
//         pcl::transformPointCloud(*pcl_vec_ori[i], *transformed_cloud, transform.matrix().cast<float>());

//         // 应用降采样
//         if (voxel_size > 0.0) {
//             pcl::VoxelGrid<PointType> voxel_filter;
//             voxel_filter.setInputCloud(transformed_cloud);
//             voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
//             voxel_filter.filter(*transformed_cloud);
//         }
        
//         // 合并点云
//         *combined_cloud += *transformed_cloud;
//     }

//     // 合并后再次降采样（可选）
//     if (voxel_size > 0.0 && !combined_cloud->empty()) {
//         pcl::VoxelGrid<PointType> final_filter;
//         final_filter.setInputCloud(combined_cloud);
//         final_filter.setLeafSize(0.2, 0.2, 0.2);
//         final_filter.filter(*combined_cloud);
//     }

//     ros::Rate loop_rate(10);
//     double duration = t;
//     double start_time = ros::Time::now().toSec();

//     while (ros::ok() && (ros::Time::now().toSec() - start_time) < duration) {
//         // 创建位姿标记（已转换到世界系）
//         visualization_msgs::MarkerArray marker_array;
//         for (size_t i = 0; i < poses_vec_ori.size(); ++i) {
//             const IMUST& pose = poses_vec_ori[i];
            
//             // 转换位姿到世界系
//             Eigen::Matrix3d R = R0_inv * pose.R;
//             Eigen::Vector3d t = R0_inv * (pose.p - p0);

//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = ros::Time::now();
//             marker.ns = "imu_poses";
//             marker.id = i;
//             marker.type = visualization_msgs::Marker::ARROW;

//             // 设置位姿
//             marker.pose.position.x = t.x();
//             marker.pose.position.y = t.y();
//             marker.pose.position.z = t.z();

//             Eigen::Quaterniond quat(R);
//             marker.pose.orientation.x = quat.x();
//             marker.pose.orientation.y = quat.y();
//             marker.pose.orientation.z = quat.z();
//             marker.pose.orientation.w = quat.w();

//             // 设置比例和颜色
//             marker.scale.x = 0.1;
//             marker.scale.y = 0.05;
//             marker.scale.z = 0.05;
//             marker.color = color;

//             marker_array.markers.push_back(marker);
//         }

//         // 创建点云消息
//         sensor_msgs::PointCloud2 pcl_msg;
//         pcl::toROSMsg(*combined_cloud, pcl_msg);
//         pcl_msg.header.frame_id = "map";
//         pcl_msg.header.stamp = ros::Time::now();

//         // 发布数据
//         pose_pub.publish(marker_array);
//         pcl_pub.publish(pcl_msg);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }
//设置可视化颜色
std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}
//获得全体位姿态向量中无人机的序号
int getDroneIndex(int search_index, std::vector<int> &pose_size_each)
{
    int drone_index = 0;
    for (int i = 0; i < pose_size_each.size(); i++)
    {
        if(search_index < pose_size_each[i])
        {
            drone_index = i;
            break;
        }
        else
        {
            search_index -= pose_size_each[i];
        }
    }
    return drone_index;
}
//获取无人机内的位姿序号
int calculatePoseNumInDroneIndex(int idx, int drone_index, const std::vector<int>& pose_size_each) {
    int pose_num_in_drone_index = idx;
    for (int k = 0; k < drone_index; k++) {
        pose_num_in_drone_index -= pose_size_each[k];
    }
    return pose_num_in_drone_index;
}
//
int getGlobalPoseIndex(int drone_index, int pose_num_in_drone_index, const std::vector<int>& pose_size_each) {
    int global_pose_index = pose_num_in_drone_index;
    for (int k = 0; k < drone_index; k++) {
        global_pose_index += pose_size_each[k];
    }
    return global_pose_index;
}


//判断是否退化
bool checkDegeneration(const Eigen::MatrixXd &Hess, double threshold) {
    Eigen::EigenSolver<Eigen::MatrixXd> solver(Hess);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
    double min_eigenvalue = eigenvalues.minCoeff();

    if (min_eigenvalue < threshold) {
        std::cout << "Degeneration detected: minimum eigenvalue = " << min_eigenvalue << std::endl;
        return true; // Degeneration detected
    } else {
        std::cout << "No degeneration: minimum eigenvalue = " << min_eigenvalue << std::endl;
        return false; // No degeneration
    }
}
// 转换机身点云到世界坐标系
void transformToGlobal(const std::vector<pcl::PointCloud<PointType>::Ptr>& body_clouds,
                       const std::vector<pose>& pose_vecs,
                       std::vector<pcl::PointCloud<PointType>::Ptr>& global_clouds) {
    global_clouds.clear();

    if (body_clouds.size() != pose_vecs.size()) {
        std::cerr << "Error: Number of point clouds and poses must match!" << std::endl;
        return;
    }

    for (size_t i = 0; i < body_clouds.size(); ++i) {
        if (!body_clouds[i]) {
            std::cerr << "Error: Null point cloud encountered at index " << i << std::endl;
            continue;
        }

        // 获取当前位姿
        const pose& current_pose = pose_vecs[i];

        // 构造世界坐标系的变换矩阵
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = current_pose.q.toRotationMatrix();  // 旋转部分
        transform.block<3, 1>(0, 3) = current_pose.t;                    // 平移部分

        // 转换点云
        pcl::PointCloud<PointType>::Ptr global_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*body_clouds[i], *global_cloud, transform);

        // 存储转换后的点云
        global_clouds.push_back(global_cloud);
    }
}
//写pose
void write_pose(std::vector<pose>& pose_vec, std::string path, int i)
{
    // 将文件后缀拼接到文件名中
    std::string filename;
    if(i==-1){
    filename = path + "pose_0.json";
    }else{
    filename = path + "pose_" + std::to_string(i) + ".json";
    }

    // 先以 trunc 模式打开文件，清空内容
    std::ofstream file;
    file.open(filename, std::ofstream::trunc);
    file.close();

    // 获取初始位姿的四元数和位移
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));

    // 以 append 模式打开文件，写入新的位姿信息
    file.open(filename, std::ofstream::app);

    for (size_t i = 0; i < pose_vec.size(); i++)
    {
        // 将姿态相对于第一个姿态进行变换
        // pose_vec[i].t << q0.inverse() * (pose_vec[i].t - t0);
        // pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
        // pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
        // pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
        // pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();

        // 写入位移和四元数
        file << pose_vec[i].t(0) << " "
            << pose_vec[i].t(1) << " "
            << pose_vec[i].t(2) << " "
            << pose_vec[i].q.w() << " " << pose_vec[i].q.x() << " "
            << pose_vec[i].q.y() << " " << pose_vec[i].q.z();

        // 如果不是最后一个姿态，添加换行符
        if (i < pose_vec.size() - 1) 
            file << "\n";
    }

    // 关闭文件
    file.close();
}
//可视化里程计轨迹
void visualizePoses(const std::vector<std::vector<pose>> &pose_vecs, const std::vector<int> &pose_sizes, ros::Publisher &marker_pub)
{
    visualization_msgs::MarkerArray marker_array;
    std::vector<std_msgs::ColorRGBA> colors = {
        [] {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
            return color;
        }(),
        [] {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            return color;
        }(),
        [] {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            return color;
        }(),
        [] {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            return color;
        }(),
        [] {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            return color;
        }(),
        [] {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
            color.a = 1.0;
            return color;
        }()
    };


    for (size_t i = 0; i < pose_vecs.size(); ++i)
    {
        const auto &poses = pose_vecs[i];
        for (size_t j = 0; j < poses.size(); ++j)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "robot_" + std::to_string(i);
            marker.id = j;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = poses[j].t.x();
            marker.pose.position.y = poses[j].t.y();
            marker.pose.position.z = poses[j].t.z();

            marker.pose.orientation.x = poses[j].q.x();
            marker.pose.orientation.y = poses[j].q.y();
            marker.pose.orientation.z = poses[j].q.z();
            marker.pose.orientation.w = poses[j].q.w();

            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;

            marker.color = colors[i % colors.size()];

            marker_array.markers.push_back(marker);
        }
    }

    marker_pub.publish(marker_array);
}

// 可视化回环做BALM球
void visualizeSpheres(const std::vector<sphere> &spheres, ros::Publisher &marker_pub, std::vector<int> loop_ba_status)
{
    for (size_t i = 0; i < spheres.size(); ++i)
    {
        const sphere &s = spheres[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "sphere";
        marker.id = i; // Unique ID for this marker
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set position
        marker.pose.position.x = s.center.x();
        marker.pose.position.y = s.center.y();
        marker.pose.position.z = s.center.z();

        // Set orientation (identity quaternion)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale (radius)
        marker.scale.x = s.radius * 2;
        marker.scale.y = s.radius * 2;
        marker.scale.z = s.radius * 2;

        // Set color (semi-transparent)
        if(loop_ba_status[i] == 0)
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
        }


        // Publish the marker
        marker_pub.publish(marker);
    }
}
void visualizeSpheres(const std::vector<sphere> &spheres, ros::Publisher &marker_pub)
{
    for (size_t i = 0; i < spheres.size(); ++i)
    {
        const sphere &s = spheres[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "sphere";
        marker.id = i; // Unique ID for this marker
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set position
        marker.pose.position.x = s.center.x();
        marker.pose.position.y = s.center.y();
        marker.pose.position.z = s.center.z();

        // Set orientation (identity quaternion)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale (radius)
        marker.scale.x = s.radius * 2;
        marker.scale.y = s.radius * 2;
        marker.scale.z = s.radius * 2;

        // Set color (semi-transparent)

    
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        


        // Publish the marker
        marker_pub.publish(marker);
    }
}
//可视化回环关系图
void visualize_loop_graph(ros::Publisher &marker_pub,
                     const std::vector<std::pair<int, int>> &start_idx,
                     const std::vector<std::pair<int, int>> &end_idx,
                     const std::vector<pose> pose_vecs[],
                     int robot_num)
{
    visualization_msgs::MarkerArray marker_array;

    // 定义颜色表，每个机器人不同颜色
    std::vector<std_msgs::ColorRGBA> colors(robot_num);
    for (int i = 0; i < robot_num; i++)
    {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = (i % 3 == 0);
        color.g = (i % 3 == 1);
        color.b = (i % 3 == 2);
        colors[i] = color;
    }

    int marker_id = 0;

    // 可视化节点
    for (size_t i = 0; i < start_idx.size(); i++)
    {
        // 开始节点
        const auto &start = start_idx[i];
        int start_robot = start.first;
        int start_pose_idx = start.second;

        const pose &start_pose = pose_vecs[start_robot][start_pose_idx];

        visualization_msgs::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = ros::Time::now();
        start_marker.ns = "start_node";
        start_marker.id = marker_id++;
        start_marker.type = visualization_msgs::Marker::SPHERE;
        start_marker.action = visualization_msgs::Marker::ADD;
        start_marker.pose.position.x = start_pose.t.x();
        start_marker.pose.position.y = start_pose.t.y();
        start_marker.pose.position.z = start_pose.t.z();
        start_marker.scale.x = 0.6;
        start_marker.scale.y = 0.6;
        start_marker.scale.z = 0.6;
        start_marker.color = colors[start_robot];
        marker_array.markers.push_back(start_marker);

        // 结束节点
        const auto &end = end_idx[i];
        int end_robot = end.first;
        int end_pose_idx = end.second;

        const pose &end_pose = pose_vecs[end_robot][end_pose_idx];

        visualization_msgs::Marker end_marker;
        end_marker.header.frame_id = "map";
        end_marker.header.stamp = ros::Time::now();
        end_marker.ns = "end_node";
        end_marker.id = marker_id++;
        end_marker.type = visualization_msgs::Marker::SPHERE;
        end_marker.action = visualization_msgs::Marker::ADD;
        end_marker.pose.position.x = end_pose.t.x();
        end_marker.pose.position.y = end_pose.t.y();
        end_marker.pose.position.z = end_pose.t.z();
        end_marker.scale.x = 0.6;
        end_marker.scale.y = 0.6;
        end_marker.scale.z = 0.6;
        end_marker.color = colors[end_robot];
        marker_array.markers.push_back(end_marker);

        // 可视化连接线
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "loop_edges";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start_point, end_point;
        start_point.x = start_pose.t.x();
        start_point.y = start_pose.t.y();
        start_point.z = start_pose.t.z();
        end_point.x = end_pose.t.x();
        end_point.y = end_pose.t.y();
        end_point.z = end_pose.t.z();

        line_marker.points.push_back(start_point);
        line_marker.points.push_back(end_point);
        line_marker.scale.x = 0.3;
        line_marker.color.a = 1.0;
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 1.0;
        marker_array.markers.push_back(line_marker);
    }

    // 发布MarkerArray
    marker_pub.publish(marker_array);
}

//计算重叠率， cloud1为当前点云，cloud2为目标点云
double calc_overlap(const pcl::PointCloud<PointType>::Ptr &cloud1,
                    const pcl::PointCloud<PointType>::Ptr &cloud2,
                    double dis_threshold,
                    pcl::PointCloud<PointType>::Ptr &overlap_points_1,
                    pcl::PointCloud<PointType>::Ptr &overlap_points_2) {
    double match_num = 0;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree(new pcl::KdTreeFLANN<PointType>);
    kd_tree->setInputCloud(cloud2);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    // 遍历 cloud1 中的所有点
    for (size_t i = 0; i < cloud1->size(); i++) {
        PointType searchPoint = cloud1->points[i];
        if (kd_tree->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < dis_threshold * dis_threshold) {
                match_num++;
                // 记录重合点到对应的云中
                overlap_points_1->push_back(searchPoint);                    // cloud1 中的点
                overlap_points_2->push_back(cloud2->points[pointIdxNKNSearch[0]]); // cloud2 中对应的点
            }
        }
    }

    std::cout << "cloud1 size:" << cloud1->size()
              << " cloud2 size: " << cloud2->size() 
              << " match size: " << match_num
              << std::endl;

    // 计算重合率
    double overlap = match_num / cloud1->size();
    return overlap;
}
// 合并点云向量到一个点云指针
pcl::PointCloud<PointType>::Ptr mergePointClouds(const std::vector<pcl::PointCloud<PointType>::Ptr>& cloud_vector) {
    pcl::PointCloud<PointType>::Ptr merged_cloud(new pcl::PointCloud<PointType>());

    for (const auto& cloud : cloud_vector) {
        if (!cloud) {
            std::cerr << "Warning: Encountered null point cloud, skipping..." << std::endl;
            continue;
        }
        *merged_cloud += *cloud;  // 使用 PCL 提供的运算符重载将点云合并
    }

    return merged_cloud;
}

//IMUST到pose的转换
void transIMUSTtoPose(std::vector<IMUST> IMUST_vec, std::vector<pose> &pose_vec) {
    // 清空输出 pose_vec，以避免重复数据
    pose_vec.clear();

    for (const auto &imu : IMUST_vec) {
        // 提取 IMUST 中的旋转矩阵 R 并转换为四元数
        Eigen::Quaterniond q(imu.R);

        // 提取平移向量 p
        Eigen::Vector3d t = imu.p;

        // 创建一个新的 pose 对象
        pose new_pose(q, t, 0); // 初始时 pose_status 设置为 0

        // 将新创建的 pose 添加到 pose_vec
        pose_vec.push_back(new_pose);
    }
}

//判断是否退化
bool isDegeneration(pcl::PointCloud<PointType>::Ptr pcl_ring, pcl::PointCloud<PointType>::Ptr pcl_sphere, double threshold){
   
    bool isDegenerate;
    int laserCloudSelNum = pcl_ring->size();
    if (laserCloudSelNum < 50) {
        return true;
    }
    // 使用Eigen定义矩阵
    Eigen::MatrixXf matA(laserCloudSelNum, 6);
    Eigen::MatrixXf matAt(6, laserCloudSelNum);
    Eigen::MatrixXf matAtA(6, 6);
    Eigen::MatrixXf matB(laserCloudSelNum, 1);
    Eigen::MatrixXf matAtB(6, 1);
    Eigen::MatrixXf matX(6, 1);

    matA.setZero();
    matAtA.setZero();
    matAtB.setZero();
    matB.setZero();
    matX.setZero();

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
        // lidar -> camera
        pointOri.x = pcl_ring->points[i].y;
        pointOri.y = pcl_ring->points[i].z;
        pointOri.z = pcl_ring->points[i].x;

        coeff.x = pcl_sphere->points[i].y;
        coeff.y = pcl_sphere->points[i].z;
        coeff.z = pcl_sphere->points[i].x;
        coeff.intensity = pcl_sphere->points[i].intensity;

        // 在不进行坐标变换的情况下，直接计算投影
        float arx = pointOri.x * coeff.x + pointOri.y * coeff.y + pointOri.z * coeff.z;
        float ary = pointOri.x * coeff.y + pointOri.y * coeff.z + pointOri.z * coeff.x;
        float arz = pointOri.x * coeff.z + pointOri.y * coeff.x + pointOri.z * coeff.y;
        // 将计算结果填充进矩阵
        matA(i, 0) = arz;
        matA(i, 1) = arx;
        matA(i, 2) = ary;
        matA(i, 3) = coeff.z;
        matA(i, 4) = coeff.x;
        matA(i, 5) = coeff.y;
        matB(i, 0) = -coeff.intensity;
    }

    matAt = matA.transpose();
    matAtA = matAt * matA;
    matAtB = matAt * matB;

    // 求解最小二乘问题
    matX = matAtA.ldlt().solve(matAtB);

    Eigen::EigenSolver<Eigen::MatrixXf> solver(matAtA);
    Eigen::MatrixXf matE = solver.eigenvalues().real();
    Eigen::MatrixXf matV = solver.eigenvectors().real();

    isDegenerate = false;

    for (int i = 5; i >= 0; i--) {
        std::cout << "特征值: " << matE(i) << std::endl;
        if (matE(i) < threshold) {
            matV.row(i).setZero();
            isDegenerate = true;
        } else {
            break;
        }
    }
    return isDegenerate;
}

void filterPointCloudbyDistance(const pcl::PointCloud<PointType>::ConstPtr &pc_in, pcl::PointCloud<PointType>::Ptr &pc_out, double threshold)
{
    double threshold_squared = threshold * threshold;

    pcl::PointIndices::Ptr indices(new pcl::PointIndices());

    for (size_t i = 0; i < pc_in->size(); i++) {
        const auto& point = pc_in->points[i];
        double dist_squared = point.y * point.y + point.z * point.z;

        if (dist_squared < threshold_squared) {
            indices->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(pc_in);
    extract.setIndices(indices);
    extract.setNegative(false);
    pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>());
    extract.filter(*new_cloud);

    pc_out = new_cloud;
}

void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                            pcl::PointCloud<PointType>& pt_out,
                            Eigen::Vector3d t,
                            Eigen::Quaterniond q)
  {
    size_t size = pc_in.points.size();
    pt_out.points.resize(size);
    for(size_t i = 0; i < size; i++)
    {
      Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      Eigen::Vector3d pt_to;
      // if(pt_cur.norm()<0.3) continue;
      pt_to = q * pt_cur + t;
      pt_out.points[i].x = pt_to.x();
      pt_out.points[i].y = pt_to.y();
      pt_out.points[i].z = pt_to.z();
      // pt_out.points[i].r = pc_in.points[i].r;
      // pt_out.points[i].g = pc_in.points[i].g;
      // pt_out.points[i].b = pc_in.points[i].b;
    }
  }

void loadPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num,
               std::string prefix = "")
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
  }
  
  void loadPCD(std::string filePath, pcl::PointCloud<PointType>::Ptr& pc, int num,
               std::string prefix = "")
  {
    std::stringstream ss;
    ss << num;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
  }

//gicp, 机身body系的点云配准
void performGICP(pcl::PointCloud<PointType>::Ptr start_frame_stack, pcl::PointCloud<PointType>::Ptr end_frame_stack, double leaf_size, Eigen::Matrix4d &final_transform){
    if (!start_frame_stack || !end_frame_stack) {
        ROS_ERROR("Input point clouds are null.");
        final_transform = Eigen::Matrix4d::Identity();
        return;
    }

    if (start_frame_stack->points.empty() || end_frame_stack->points.empty()) {
        ROS_ERROR("Input point clouds are empty.");
        final_transform = Eigen::Matrix4d::Identity();
        return;
    }
    
    pcl::PointCloud<PointType>::Ptr start_filtered(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr end_filtered(new pcl::PointCloud<PointType>);
    
    pcl::copyPointCloud(*start_frame_stack, *start_filtered);
    pcl::copyPointCloud(*end_frame_stack, *end_filtered);
    
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.setInputCloud(start_filtered);
    sor.filter(*start_filtered);
    sor.setInputCloud(end_filtered);
    sor.filter(*end_filtered);

    if (start_filtered->points.empty() || end_filtered->points.empty()) {
        ROS_ERROR("Filtered point cloud is empty after preprocessing!");
        pcl::copyPointCloud(*start_frame_stack, *start_filtered);
        pcl::copyPointCloud(*end_frame_stack, *end_filtered);
    }
    
    down_sampling_voxel(*start_filtered, leaf_size);
    down_sampling_voxel(*end_filtered, leaf_size);
    
    double avg_dist = 0.0;
    double threshold = 0.0;
    
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(end_filtered);
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    
    double sum_dist = 0.0;
    int valid_points = 0;
    
    for (size_t i = 0; i < start_filtered->points.size(); ++i) {
        if (kdtree.nearestKSearch(start_filtered->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            sum_dist += sqrt(pointNKNSquaredDistance[0]);
            valid_points++;
        }
    }
    
    if (valid_points > 0) {
        avg_dist = sum_dist / valid_points;
        threshold = avg_dist * 2.5;
        if (threshold > 25.0) {
            ROS_INFO("Threshold %f is too large, using defualt threshold.", threshold);
            threshold = 1.0;
        } else if (threshold < 1.0) {
            ROS_INFO("Threshold %f is too small, using defualt threshold.", threshold);
            threshold = 1.0;
        }
    } else {
        threshold = 1.0;
        ROS_INFO("Not enough valid points found for distance calculation, using default threshold.");
    }
    
    ROS_INFO("Adaptive threshold for GICP: %f", threshold);
    
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(end_filtered);
    gicp.setInputTarget(start_filtered);
    gicp.setMaxCorrespondenceDistance(threshold);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    
    pcl::PointCloud<PointType> aligned;
    gicp.align(aligned);
    
    if(!gicp.hasConverged()) {
        ROS_ERROR("GICP not converge!");
        final_transform = Eigen::Matrix4d::Identity();
        return;
    } else {
        ROS_INFO("GICP score: %.2f", gicp.getFitnessScore());
    }

    final_transform = gicp.getFinalTransformation().cast<double>();
}

//icp, 机身body系的点云配准
void performICP(pcl::PointCloud<PointType>::Ptr start_frame_stack, pcl::PointCloud<PointType>::Ptr end_frame_stack, double leaf_size, Eigen::Matrix4d &final_transform){
    if (!start_frame_stack || !end_frame_stack) {
        ROS_ERROR("Input point clouds are null.");
        final_transform = Eigen::Matrix4d::Identity();
        return;
    }

    if (start_frame_stack->points.empty() || end_frame_stack->points.empty()) {
        ROS_ERROR("Input point clouds are empty.");
        final_transform = Eigen::Matrix4d::Identity();
        return;
    }
    
    pcl::PointCloud<PointType>::Ptr start_filtered(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr end_filtered(new pcl::PointCloud<PointType>);
    
    pcl::copyPointCloud(*start_frame_stack, *start_filtered);
    pcl::copyPointCloud(*end_frame_stack, *end_filtered);
    
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.setInputCloud(start_filtered);
    sor.filter(*start_filtered);
    sor.setInputCloud(end_filtered);
    sor.filter(*end_filtered);

    if (start_filtered->points.empty() || end_filtered->points.empty()) {
        ROS_ERROR("Filtered point cloud is empty after preprocessing!");
        pcl::copyPointCloud(*start_frame_stack, *start_filtered);
        pcl::copyPointCloud(*end_frame_stack, *end_filtered);
    }
    
    down_sampling_voxel(*start_filtered, leaf_size);
    down_sampling_voxel(*end_filtered, leaf_size);
    
    double avg_dist = 0.0;
    double threshold = 0.0;
    
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(start_filtered);
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    
    double sum_dist = 0.0;
    int valid_points = 0;
    
    for (size_t i = 0; i < end_filtered->points.size(); ++i) {
        if (kdtree.nearestKSearch(end_filtered->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            sum_dist += sqrt(pointNKNSquaredDistance[0]);
            valid_points++;
        }
    }
    
    if (valid_points > 0) {
        avg_dist = sum_dist / valid_points;
        threshold = avg_dist * 2.5;
        if (threshold > 25.0) {
            ROS_INFO("Threshold %f is too large, using defualt threshold.", threshold);
            threshold = 1.0;
        } else if (threshold < 1.0) {
            ROS_INFO("Threshold %f is too small, using defualt threshold.", threshold);
            threshold = 1.0;
        }
    } else {
        threshold = 1.0;
        ROS_INFO("No valid points found for distance calculation, using default threshold.");
    }
    
    ROS_INFO("Adaptive threshold for ICP: %f", threshold);
    
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(end_filtered);
    icp.setInputTarget(start_filtered);
    icp.setMaxCorrespondenceDistance(threshold);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    
    pcl::PointCloud<PointType> aligned;
    icp.align(aligned);
    
    if(!icp.hasConverged()) {
        ROS_INFO("ICP not converge, try GICP.");
        performGICP(start_frame_stack, end_frame_stack, leaf_size, final_transform);
        return;
    } else {
        ROS_INFO("ICP score: %.2f", icp.getFitnessScore());
    }

    final_transform = icp.getFinalTransformation().cast<double>();
}

// 计算两点间距离
double distance(const pos& a, const pos& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + 
           std::pow(a.y - b.y, 2) + 
           std::pow(a.z - b.z, 2));
}

// 聚类函数
void clusterLoops(const std::vector<pos>& loop_center,
                  std::vector<int>& loop_BA_type,
                  std::vector<int>& isolated_loops,
                  std::vector<std::vector<int>>& cluster_loops,
                  double normal_radius) 
{
    const int loop_num = loop_center.size();
    std::vector<bool> visited(loop_num, false);
    loop_BA_type.resize(loop_num, 0);

    std::cout << "======== 开始回环聚类 ========" << std::endl;
    std::cout << "总回环数: " << loop_num << std::endl;
    std::cout << "邻域半径: " << normal_radius << " 米" << std::endl;

    for (int i = 0; i < loop_num; ++i) {
        if (!visited[i]) {
            std::vector<int> current_cluster;
            std::queue<int> search_queue;
            
            search_queue.push(i);
            visited[i] = true;

            std::cout << "\n启动新聚类搜索，种子点: " << i << std::endl;

            while (!search_queue.empty()) {
                int current_idx = search_queue.front();
                search_queue.pop();
                current_cluster.push_back(current_idx);

                std::cout << " 处理点 " << current_idx 
                          << " (" << loop_center[current_idx].x << ", "
                          << loop_center[current_idx].y << ")" 
                          << std::endl;

                // 寻找邻域点
                for (int j = 0; j < loop_num; ++j) {
                    if (!visited[j] && j != current_idx) {
                        double dist = distance(loop_center[current_idx], 
                                             loop_center[j]);
                        if (dist <= 1.8 * normal_radius) {
                            visited[j] = true;
                            search_queue.push(j);
                            
                            std::cout << "  发现邻域点 " << j 
                                      << " 距离: " << dist << " 米" 
                                      << std::endl;
                        }
                    }
                }
            }

            // 分类结果
            if (current_cluster.size() == 1) {
                isolated_loops.push_back(current_cluster[0]);
                loop_BA_type[current_cluster[0]] = 0;
                
                std::cout << ">> 孤立回环: " << current_cluster[0] 
                          << std::endl;
            } else {
                cluster_loops.push_back(current_cluster);
                for (int idx : current_cluster) {
                    loop_BA_type[idx] = 1;
                }
                
                std::cout << ">> 发现聚类 " << cluster_loops.size()
                          << " 包含 " << current_cluster.size() 
                          << " 个回环" << std::endl;
            }
        }
    }

    // 打印汇总信息
    std::cout << "\n====== 聚类结果汇总 ======" << std::endl;
    std::cout << "孤立回环数: " << isolated_loops.size() << std::endl;
    std::cout << "聚类数量: " << cluster_loops.size() << std::endl;
    
    for (size_t i = 0; i < cluster_loops.size(); ++i) {
        std::cout << "聚类 " << i+1 << " ("
                  << cluster_loops[i].size() << " 个): ";
        for (int idx : cluster_loops[i]) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "孤立回环列表: ";
    for (int idx : isolated_loops) {
        std::cout << idx << " ";
    }
    std::cout << "\n==========================" << std::endl;
}

// 可视化函数
void visualizePoseClusters(ros::Publisher& pub, std::vector<pose>* cluster_poses, int cluster_num) {
    visualization_msgs::Marker points;
    points.header.frame_id = "map"; // 修改为你的固定坐标系
    points.header.stamp = ros::Time::now();
    points.ns = "pose_clusters";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points.scale.z = 0.3; // 文字大小
    points.color.r = 1.0;
    points.color.g = 1.0;
    points.color.b = 1.0;
    points.color.a = 1.0;

    int index = 1; // 索引从 1 开始
    for (int i = 0; i < cluster_num; i++) {
        for (size_t j = 0; j < cluster_poses[i].size(); j++) {
            visualization_msgs::Marker text_marker = points;
            text_marker.id = index;
            text_marker.pose.position.x = cluster_poses[i][j].t.x();
            text_marker.pose.position.y = cluster_poses[i][j].t.y();
            text_marker.pose.position.z = cluster_poses[i][j].t.z();
            text_marker.text = std::to_string(index);
            pub.publish(text_marker);
            index++;
        }
    }
}

// 计算均值
pos computeMean(const std::vector<pos>& points) {
    pos mean = {0, 0, 0};
    int n = points.size();
    for (const auto& p : points) {
        mean.x += p.x;
        mean.y += p.y;
        mean.z += p.z;
    }
    mean.x /= n;
    mean.y /= n;
    mean.z /= n;
    return mean;
}

// 计算协方差矩阵并获取主方向
Eigen::Vector3d computePrincipalDirection(const std::vector<pos>& points) {
    int n = points.size();
    if (n < 2) return Eigen::Vector3d(1, 0, 0); // 只有一个点时，默认x方向

    pos mean = computeMean(points);

    // 计算协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d diff(p.x - mean.x, p.y - mean.y, p.z - mean.z);
        cov += diff * diff.transpose();
    }
    cov /= n;

    // 计算特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d principalDirection = solver.eigenvectors().col(2); // 取最大特征值对应的特征向量

    return principalDirection;
}

// 根据主方向排序
void sortClusterByPrincipalDirection(std::vector<int>& cluster, const std::vector<pos>& loop_center) {
    std::vector<pos> cluster_points;
    for (int idx : cluster) {
        cluster_points.push_back(loop_center[idx]);
    }

    Eigen::Vector3d direction = computePrincipalDirection(cluster_points);
    pos mean = computeMean(cluster_points);

    // 按照主方向投影值排序
    std::sort(cluster.begin(), cluster.end(), [&](int a, int b) {
        Eigen::Vector3d pa(loop_center[a].x - mean.x, loop_center[a].y - mean.y, loop_center[a].z - mean.z);
        Eigen::Vector3d pb(loop_center[b].x - mean.x, loop_center[b].y - mean.y, loop_center[b].z - mean.z);
        return pa.dot(direction) < pb.dot(direction);
    });
}

// 处理所有回环聚类
void processClusters(std::vector<std::vector<int>>& cluster_loops, const std::vector<pos>& loop_center) {
    for (auto& cluster : cluster_loops) {
        if (cluster.size() > 1) {
            sortClusterByPrincipalDirection(cluster, loop_center);
        }
    }
}

// 计算均值中心
Eigen::Vector3d computePoseMean(const std::vector<pose>& poses) {
    Eigen::Vector3d mean(0, 0, 0);
    int n = poses.size();
    for (const auto& p : poses) {
        mean += p.t;
    }
    mean /= n;
    return mean;
}

// 计算协方差矩阵并获取主方向
Eigen::Vector3d computePosePrincipalDirection(const std::vector<pose>& poses) {
    int n = poses.size();
    if (n < 2) return Eigen::Vector3d(1, 0, 0);  // 只有一个点时，默认x方向

    Eigen::Vector3d mean = computePoseMean(poses);

    // 计算协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : poses) {
        Eigen::Vector3d diff = p.t - mean;
        cov += diff * diff.transpose();
    }
    cov /= n;

    // 计算特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d principalDirection = solver.eigenvectors().col(2); // 取最大特征值对应的特征向量

    return principalDirection;
}

// 按主方向排序，同时记录索引
void sortPoseClusterByPrincipalDirection(std::vector<pose>& cluster, std::vector<IMUST>& imusts) {
    if (cluster.size() < 2) return;

    Eigen::Vector3d direction = computePosePrincipalDirection(cluster);
    Eigen::Vector3d mean = computePoseMean(cluster);

    // 记录原始索引
    std::vector<int> indices(cluster.size());
    for (int i = 0; i < cluster.size(); i++) indices[i] = i;

    // 按主方向投影值排序
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        double proj_a = (cluster[a].t - mean).dot(direction);
        double proj_b = (cluster[b].t - mean).dot(direction);
        return proj_a < proj_b;
    });

    // 使用排序后的索引重新排列
    std::vector<pose> sorted_cluster(cluster.size());
    std::vector<IMUST> sorted_imusts(imusts.size());
    for (int i = 0; i < cluster.size(); i++) {
        sorted_cluster[i] = cluster[indices[i]];
        sorted_imusts[i] = imusts[indices[i]];
    }

    cluster = sorted_cluster;
    imusts = sorted_imusts;
}

// 处理所有聚类
void processPoseClusters(std::vector<pose>* cluster_poses, std::vector<IMUST>* cluster_imust, int cluster_num) {
    for (int i = 0; i < cluster_num; i++) {
        if (cluster_poses[i].size() > 1) {
            sortPoseClusterByPrincipalDirection(cluster_poses[i], cluster_imust[i]);
        }
    }
}

int getMinIndex(const std::vector<IMUST>& imusts){
    int minIndex = 0;
    double minT = imusts[0].t;
    for(int i = 1; i < imusts.size(); i++){
        if(imusts[i].t < minT){
            minT = imusts[i].t;
            minIndex = i;
        }
    }
    return minT;
}

int getMaxIndex(const std::vector<IMUST>& imusts){
    int maxIndex = 0;
    double maxT = imusts[0].t;
    for(int i = 1; i < imusts.size(); i++){
        if(imusts[i].t > maxT){
            maxT = imusts[i].t;
            maxIndex = i;
        }
    }
    return maxT;
}

IMUST getNearestIMUST(const std::vector<IMUST>& imusts, IMUST imust){
    double minDist = 100000;
    IMUST nearestIMUST;
    for(int i = 0; i < imusts.size(); i++){
        double dist = std::sqrt(std::pow(imust.p[0]- imusts[i].p[0], 2) + std::pow(imust.p[1] - imusts[i].p[1], 2) + std::pow(imust.p[2] - imusts[i].p[2], 2));
        if(dist < minDist){
            minDist = dist;
            nearestIMUST = imusts[i];
        }
    }
    return nearestIMUST;
}