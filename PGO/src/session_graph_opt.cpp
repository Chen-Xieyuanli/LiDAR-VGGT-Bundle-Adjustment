#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>  // Include for TF broadcasting
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include <iomanip>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <gtsam/geometry/Pose3.h> // GTSAM 位姿表示
#include <gtsam/slam/PriorFactor.h> // GTSAM 先验因子
#include <gtsam/slam/BetweenFactor.h> // GTSAM 两帧之间的因子
#include <gtsam/nonlinear/Values.h> // GTSAM 非线性方程组的结果
#include <gtsam/nonlinear/ISAM2.h> // GTSAM 增量式优化

using namespace std;

/***********************************************************************************************************************************/
/************输入：VGGT 的每一个session的 所有位姿和body系的点云**************************************************************************/
/************输出：优化后每一个session的位姿 每一个session的PGO后的世界系点云***************************************************************/
/***********************************************************************************************************************************/

// pose结构体  位姿，时间戳，session_id
struct pose
{
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
            Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)):q(_q), t(_t){}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    double timestamp;
    int session_id;   // Jack 记录当前位姿属于哪个session
};

// Jack 读取session内的pose 记录了每个pose的session_id
int read_TUM_pose(std::string filename, std::vector<pose> &pose_vec, int session_id) {
    std::fstream file;
    file.open(filename);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    while (file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        Eigen::Quaterniond qd(qw, qx, qy, qz);
        Eigen::Vector3d td(tx, ty, tz);
        pose pose_tmp;
        pose_tmp.q = qd;
        pose_tmp.t = td;
        pose_tmp.timestamp = timestamp;
        pose_tmp.session_id = session_id;
        pose_vec.push_back(pose_tmp);
    }
    return pose_vec.size();
}

// 读取session的overlap点云部分 VGGT的RGB点云转为XYZ点云
// 从start_idx 开始读取，读取 overlap_num 个body点云 
void read_overlap_pcd(std::string filename, int session_id, int overlap_num, int start_idx, 
                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pcd_vec) {
    for (int i = start_idx; i < overlap_num + start_idx; i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string pcd_file = filename + std::to_string(session_id) + "/camera/pcd/" + std::to_string(i) + ".pcd";
        // std::cout << "Reading PCD file: " << pcd_file << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> pl_tem;
        pcl::io::loadPCDFile(pcd_file, pl_tem);
        for(const auto& point : pl_tem.points) {
            pcl::PointXYZ p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloud->points.push_back(p);
        }
        pcd_vec.push_back(cloud);
    }
}

// 读取session的overlap点云部分 VGGT的RGB点云
// 从start_idx 开始读取，读取 overlap_num 个body点云
void read_overlap_pcd_rgb(std::string filename, int session_id, int overlap_num, int start_idx, 
                      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcd_vec) {
    for (int i = start_idx; i < overlap_num + start_idx; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string pcd_file = filename + std::to_string(session_id) + "/camera/pcd/" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(pcd_file, *cloud);
        pcd_vec.push_back(cloud);
    }
}

// 读取每个session的所有RGB点云 num个
void read_all_pcd_rgb(std::string filename, int session_id, int num, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcd_vec) {
    for (int i = 0; i < num; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string pcd_file = filename + std::to_string(session_id) + "/camera/pcd/" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(pcd_file, *cloud);
        pcd_vec.push_back(cloud);
    }
}

// 转换到世界系
pcl::PointCloud<pcl::PointXYZ>::Ptr mergeLidarPointsToWorldFrame(
    const std::vector<pose>& lidar_pose,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& lidar_points)
{
    if (lidar_pose.empty() || lidar_points.empty()) {
        std::cout << "Input lidar_pose or lidar_points is empty!" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    size_t n_pose = lidar_pose.size();
    size_t n_cloud = lidar_points.size();
    if (n_pose != n_cloud) {
        std::cout << "Warning: lidar_pose.size() != lidar_points.size(), using min(): "
                  << n_pose << " vs " << n_cloud << std::endl;
    }
    size_t n = std::min(n_pose, n_cloud);
    if (n == 0) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points_all(new pcl::PointCloud<pcl::PointXYZ>());

    for (size_t i = 0; i < n; ++i) {
        const auto& pose_i = lidar_pose[i];
        const auto& cloud = lidar_points[i];
        Eigen::Matrix3d R = pose_i.q.toRotationMatrix();
        Eigen::Vector3d t = pose_i.t;

        for (const auto& pt : cloud->points) {
            Eigen::Vector3d p_body(pt.x, pt.y, pt.z);
            // 直接投影到世界系
            Eigen::Vector3d p_world = R * p_body + t;

            pcl::PointXYZ new_pt;
            new_pt.x = static_cast<float>(p_world.x());
            new_pt.y = static_cast<float>(p_world.y());
            new_pt.z = static_cast<float>(p_world.z());
            lidar_points_all->points.push_back(new_pt);
        }
    }

    lidar_points_all->width = static_cast<uint32_t>(lidar_points_all->points.size());
    lidar_points_all->height = 1;
    lidar_points_all->is_dense = false;

    return lidar_points_all;
}

// RGB点云转换到世界系
pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeLidarPointsToWorldFrameRGB(
    const std::vector<pose>& lidar_pose,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& lidar_points)
{
    if (lidar_pose.empty() || lidar_points.empty()) {
        std::cout << "Input lidar_pose or lidar_points is empty!" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    size_t n = std::min(lidar_pose.size(), lidar_points.size());
    if (n == 0) {
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_points_all(new pcl::PointCloud<pcl::PointXYZRGB>());
    size_t total_points = 0;
    for (size_t i = 0; i < n; ++i) total_points += lidar_points[i]->size();
    lidar_points_all->points.reserve(total_points);

    for (size_t i = 0; i < n; ++i) {
        const auto& pose_i = lidar_pose[i];
        const auto& cloud = lidar_points[i];
        Eigen::Matrix3d R = pose_i.q.toRotationMatrix();
        Eigen::Vector3d t = pose_i.t;

        for (const auto& pt : cloud->points) {
            Eigen::Vector3d p_body(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_world = R * p_body + t;

            pcl::PointXYZRGB new_pt;
            new_pt.x = static_cast<float>(p_world.x());
            new_pt.y = static_cast<float>(p_world.y());
            new_pt.z = static_cast<float>(p_world.z());
            new_pt.r = pt.r;
            new_pt.g = pt.g;
            new_pt.b = pt.b;
            lidar_points_all->points.push_back(new_pt);
        }
    }

    lidar_points_all->width = static_cast<uint32_t>(lidar_points_all->points.size());
    lidar_points_all->height = 1;
    lidar_points_all->is_dense = false;

    return lidar_points_all;
}


// Jack overlap区域icp配准
gtsam::Pose3 overlap_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target){
    // ICP 配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(400);
    // icp.setMaxCorrespondenceDistance(3.0); // 设置最大对应点距离
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    
    gtsam::Pose3 T_icp;


    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl;
        std::cout << "The score is " << icp.getFitnessScore() << std::endl;
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << transformation << std::endl;

        // 将 Eigen::Matrix4f 转换为 gtsam::Pose3
        Eigen::Matrix3d R = transformation.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d t = transformation.block<3, 1>(0, 3).cast<double>();

        T_icp = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(t));
        return T_icp;
    }
    else{
        std::cout << "ICP did not converge." << std::endl;
        return T_icp;
    }
}

// 下采样
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVoxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
    float voxel_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

// Jack: 发布点云到 RViz
void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const ros::Publisher& pub,
                       const std::string& frame_id,
                       const std::string& topic_name) 
{
    if (cloud->empty()) {
        std::cout << "Warning: " << topic_name << " cloud is empty, skip publishing." << std::endl;
        return;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;  // eg: "map" 或 "world"
    pub.publish(msg);
    std::cout << "Published " << topic_name << " with " << cloud->points.size() << " points." << std::endl;
}


// 保存TUM格式的pose
void save_TUM_Pose(const std::string &file_path,
               const std::vector<pose> &poses)
{
    std::ofstream pose_file(file_path);

    for (const auto &pose : poses) {
      //change R to qx qy qz qw
      Eigen::Quaterniond q = pose.q;
      // change p to x y z
      //高小数精度
      pose_file << std::fixed << std::setprecision(8) << pose.timestamp << " " 
      << std::fixed << std::setprecision(6) << pose.t.x() << " " 
      << std::fixed << std::setprecision(6) << pose.t.y() << " " 
      << std::fixed << std::setprecision(6) << pose.t.z() << " " 
      << std::fixed << std::setprecision(6) << q.x() << " " 
      << std::fixed << std::setprecision(6) << q.y() << " " 
      << std::fixed << std::setprecision(6) << q.z() << " " 
      << std::fixed << std::setprecision(6) << q.w() << std::endl;
    }
    pose_file.close();
}

//设置可视化颜色
std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// Jack 保存彩色的点云 ply格式
void saveColoredPointCloud(const std::string &file_path,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb) {
    if (pcl::io::savePCDFileBinary(file_path, *cloud_rgb) == -1) {
        PCL_ERROR("Failed to save PCD file %s\n", file_path.c_str());
    } else {
        std::cout << "Saved colored point cloud to " << file_path
                  << " with " << cloud_rgb->points.size() << " points." << std::endl;
    }
}

//Jack session 因子图优化
void session_pgo(vector<pose> vec_pose, 
                 int session_overlap,
                 int session_num,
                 int session_size,
                 gtsam::Values &initial,
                 gtsam::NonlinearFactorGraph &graph,
                 vector<int> pose_size_each,
                 vector<pose> &pose_opt,
                 vector<gtsam::Pose3> T_icp){
    
    // 设置先验置信度
    gtsam::Vector Prior_Vector6(6);
    Prior_Vector6 << 1e-12, 1e-12, 1e-12, 1e-14, 1e-14, 1e-14;    // x,y,z,roll,pitch,yaw
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Prior_Vector6);

    // 设置帧间估计置信度
    gtsam::Vector odom_Vector6(6);
    odom_Vector6 << 1e-8, 1e-8, 1e-8, 1e-10, 1e-10, 1e-10;
    gtsam::noiseModel::Diagonal::shared_ptr odomModel = gtsam::noiseModel::Diagonal::Variances(odom_Vector6);

    // 设置session间 icp的置信度
    gtsam::Vector icp_Vector6(6);
    icp_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr icpModel = gtsam::noiseModel::Diagonal::Variances(icp_Vector6);

    // 记录不同相邻的session
    std::vector<int> session_vector;
    int len = 0;
    for(int i = 0; i < pose_size_each.size(); i++){
        len += pose_size_each[i];
        session_vector.push_back(len);
    }
    
    // 添加节点和边
    for(int i = 0; i < vec_pose.size(); i++){
        bool addFlag = true;
        // 第一个位姿
        if(i == 0){
            // 插入当前位姿到初始估计中
            if (!initial.exists(i)) {
                initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));
            }
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)), priorModel));
            continue;
        }
        else{ //其他位姿
            if (!initial.exists(i)) {
                initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));
            }
            // 计算相邻帧之间的相对位姿
            gtsam::Pose3 newPose(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t));
            gtsam::Pose3 oldPose(gtsam::Rot3(vec_pose[i-1].q.toRotationMatrix()), gtsam::Point3(vec_pose[i-1].t));
            gtsam::Pose3 T_new_old = oldPose.between(newPose);

            //判断是否添加
            for(int k = 0; k < session_vector.size(); k++){
                if(i == session_vector[k]){
                    addFlag = false;
                }
            }
            if(addFlag){
                // 添加帧间约束
                graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, T_new_old, odomModel));
            }
        }
    }

    // 添加session间的icp约束
    for(int i = 0; i < session_num - 1; i++){
        int source_idx = (i + 1) * session_size;
        int target_idx = source_idx - session_overlap; 
        std::cout << "Adding ICP constraint between pose " << target_idx << " and pose " << source_idx << std::endl;
        graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(source_idx, target_idx, T_icp[i], icpModel)); // Jack
    }

    // 优化参数设置
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.001;
    parameters.relinearizeSkip = 1;

    // 创建优化器
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for (int i = 0; i < 10; i++) {
        isam.update();
    }

    // 计算优化结果
    gtsam::Values results = isam.calculateEstimate();

    // 提取优化后的位姿
    pose_opt.reserve(vec_pose.size());
    for (int i = 0; i < results.size(); i++) {
        gtsam::Pose3 pose_gtsam = results.at<gtsam::Pose3>(i);
        pose tmp;
        tmp.q = Eigen::Quaterniond(pose_gtsam.rotation().matrix());
        tmp.t = pose_gtsam.translation();
        tmp.timestamp = vec_pose[i].timestamp;
        pose_opt.push_back(tmp);
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "session_graph_opt");
    ros::NodeHandle nh("~");
    /***************    read poses ---> overlap区域icp收集约束 ---> Factor Graph Optimization --> save optimized chunk ply(世界系的) *********************/

    //点云pcd和位姿pose的路径
    std::string base_data_path;
    nh.getParam("base_data_path", base_data_path);

    //VGGT session 相关参数
    int session_num;
    int session_size;
    int session_overlap;
    nh.getParam("session_num", session_num);
    nh.getParam("session_size", session_size);
    nh.getParam("session_overlap", session_overlap);

    //降采样
    double voxel_size;
    nh.getParam("voxel_size", voxel_size);

    //保存的chunk pcd
    std::string save_pcd_path;
    nh.getParam("save_pcd_path", save_pcd_path);

    //可视化 RViz ，优化前后的点云和位姿
    ros::Publisher pose_pub_initial = nh.advertise<visualization_msgs::MarkerArray>("/poses_initial", 100);
    ros::Publisher pose_pub_opt = nh.advertise<visualization_msgs::MarkerArray>("/poses_opt", 100);

    ros::Publisher source_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_source_down", 1);
    ros::Publisher target_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_target_down", 1);
    ros::Publisher icp_source_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_source_aligned", 1);

    //可视化颜色
    std_msgs::ColorRGBA green = createColorRGBA(0.0, 1.0, 0.0, 1.0);
    std_msgs::ColorRGBA red = createColorRGBA(1.0, 0.0, 0.0, 1.0);
    std_msgs::ColorRGBA blue = createColorRGBA(0.0, 0.0, 1.0, 1.0);

    //PGO output，每一个chunk单独输出一个pose文件   
    std::string output_path_each[session_num];
    for(int i = 0; i < session_num; i++){
        output_path_each[i] = base_data_path + std::to_string(i) + "/camera/poses_pgo.txt";
    }

    // pose vector，multi-session
    std::vector<pose> pose_vecs[session_num];    
    std::vector<int> pose_size_each(session_num);
    int pose_total = 0; 
    std::vector<pose> pose_vec_total;
    
    // 读取VGGT session 的pose
    // 参与PGO优化的量： pose_vec_total
    
    for (int i = 0; i < session_num; i++)
    {
        std::vector<pose> pose_vec;
        int pose_size = read_TUM_pose(base_data_path + std::to_string(i) + "/camera/poses.txt", pose_vec, i);
        pose_vecs[i] = pose_vec;
        pose_size_each[i] = pose_size;
        pose_total += pose_size;
        pose_vec_total.insert(pose_vec_total.end(), pose_vec.begin(), pose_vec.end());
    }
    std::cout << "Total pose size: " << pose_vec_total.size() << std::endl;

    //获取所有的overlap区域的icp约束
    std::vector<gtsam::Pose3> T_start_end;   // 后一个到前一个的变换 source是后一个session, 匹配到前一个session target 接近单位阵列
    // 前一次source的变换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr next_target(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr next_target_down(new pcl::PointCloud<pcl::PointXYZ>);
    // target: 前一个session
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_all_down;
    for(int i = 0; i < session_num - 1; i++){

        pcl::PointCloud<pcl::PointXYZ>::Ptr align_source(new pcl::PointCloud<pcl::PointXYZ>);
        std::cout << "Processing overlap between session " << i << " and session " << i + 1 << std::endl;

        // source: 后一个session 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcd_vec_source;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcd_vec_source_rgb;
        int start_idx_source = 0;
        int source_overlap_num = std::min(session_overlap, pose_size_each[i + 1]);
        read_overlap_pcd(base_data_path, i + 1, source_overlap_num, start_idx_source, pcd_vec_source);
        // read_overlap_pcd_rgb(base_data_path, i + 1, source_overlap_num, start_idx_source, pcd_vec_source_rgb);
        read_all_pcd_rgb(base_data_path, i + 1, pose_size_each[i + 1], pcd_vec_source_rgb);
        std::vector<pose> pose_vec_source(pose_vecs[i + 1].begin(), pose_vecs[i + 1].begin() + source_overlap_num);
        std::cout << "pcd_vec_source size: " << pcd_vec_source.size() << "  pose_vec_source size: " << pose_vec_source.size() << std::endl;
        std::cout << "mergeLidarPointsToWorldFrame...source" << std::endl;                                 // 转换到body系
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_all = mergeLidarPointsToWorldFrame(pose_vec_source, pcd_vec_source);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_all_rgb = mergeLidarPointsToWorldFrameRGB(pose_vecs[i + 1], pcd_vec_source_rgb);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_all_down;
        source_all_down = downsampleVoxel(source_all, voxel_size);
        publishPointCloud(source_all_down, source_pub, "map", "source_all_down");


         
        if(i == 0){
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcd_vec_target;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcd_vec_target_rgb;
            int start_idx_target = session_size - session_overlap;
            std::cout << "start_idx_target: " << start_idx_target << std::endl;
            read_overlap_pcd(base_data_path, i, session_overlap, start_idx_target, pcd_vec_target);
            // read_overlap_pcd_rgb(base_data_path, i, session_overlap, start_idx_target, pcd_vec_target_rgb);
            read_all_pcd_rgb(base_data_path, i, pose_size_each[i], pcd_vec_target_rgb);
            std::vector<pose> pose_vec_target(pose_vecs[i].begin() + start_idx_target, pose_vecs[i].end());
            std::cout << "pcd_vec_target size: " << pcd_vec_target.size() << "  pose_vec_target size: " << pose_vec_target.size() << std::endl;
            std::cout << "mergeLidarPointsToWorldFrame...target" << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_all = mergeLidarPointsToWorldFrame(pose_vec_target, pcd_vec_target);
            target_all_down = downsampleVoxel(target_all, voxel_size);
            std::cout << "source_all_down size: " << source_all_down->points.size() << "  target_all_down size: " << target_all_down->points.size() << std::endl;
            publishPointCloud(target_all_down, target_pub, "map", "target_all_down");
            std::cout << "First session, save as reference." << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_all_rgb = mergeLidarPointsToWorldFrameRGB(pose_vecs[i], pcd_vec_target_rgb);
            saveColoredPointCloud(save_pcd_path + "session_0.pcd", target_all_rgb);
        }
        else{
            // target_all_down.reset(new pcl::PointCloud<pcl::PointXYZ>(*next_target_down));
             // 马老师说：停停
            ros::Duration(5.0).sleep();  // 睡眠5秒，确保点云发布完成
            std::cout << "====================================================================" << std::endl;
            publishPointCloud(target_all_down, target_pub, "map", "target_all_down");
            std::cout << "Not first session, use last source as target." << std::endl;
        }

        // icp配准
        std::cout << "performing icp..." << std::endl;
        gtsam::Pose3 T_icp = overlap_icp(source_all_down, target_all_down);
        T_start_end.push_back(T_icp);

        // 转换为Eigen矩阵
        // 转换为Eigen矩阵
        Eigen::Matrix4f T_eigen = Eigen::Matrix4f::Identity();
        T_eigen.block<3,3>(0,0) = T_icp.rotation().matrix().cast<float>();
        T_eigen.block<3,1>(0,3) = T_icp.translation().cast<float>();   // ✅ 直接 cast 就行


        // 生成对齐后的点云
        pcl::transformPointCloud(*source_all_down, *align_source, T_eigen);
        publishPointCloud(align_source, icp_source_aligned_pub, "map", "icp_source_aligned_" + std::to_string(i));

        // 生成 下次的target
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcd_vec_next_target;
        int start_idx_next_target = pose_size_each[i + 1] - session_overlap;
        int next_target_overlap_num = std::min(session_overlap, pose_size_each[i + 1]);
        read_overlap_pcd(base_data_path, i + 1, next_target_overlap_num, start_idx_next_target, pcd_vec_next_target);
        std::vector<pose> pose_vec_next_target(pose_vecs[i + 1].begin() + start_idx_next_target, pose_vecs[i + 1].end());
        next_target = mergeLidarPointsToWorldFrame(pose_vec_next_target, pcd_vec_next_target);
        // target_all_down.clea
        target_all_down = downsampleVoxel(next_target, voxel_size);

        // 马老师说：停停
        ros::Duration(5.0).sleep();  // 睡眠5秒，确保点云发布完成
        std::cout << "ICP between session " << i << " and session " << i + 1 << " done." << std::endl;

        // 保存icp配准后的source RGB点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_rgb_save(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*source_all_rgb, *source_rgb_save, T_eigen);
        saveColoredPointCloud(save_pcd_path + "session_" + std::to_string(i + 1) + ".pcd", source_rgb_save);

    }

    std::cout << "Total ICP constraints: " << T_start_end.size() << "  Session num: " << session_num << std::endl;

    //因子图优化
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    std::vector<pose> pose_opt_total;
    std::vector<pose> pose_opt_each_vec[session_num];
    session_pgo(pose_vec_total, session_overlap, session_num, session_size, initial, graph, pose_size_each, pose_opt_total, T_start_end);

    // 分解成每个session优化后的位姿
    int pose_index = 0;
    for(int i = 0; i < session_num; i++)
    {
        pose_opt_each_vec[i].insert(pose_opt_each_vec[i].end(), pose_opt_total.begin() + pose_index, pose_opt_total.begin() + pose_index + pose_size_each[i]);
        pose_index += pose_size_each[i];
    }
    pose_opt_total.clear();

    //保存优化后的位姿
    for(int i = 0; i < session_num; i++)
    {   
        std::cout << "Writing" << i <<  " to : " << output_path_each[i] << std::endl;
        save_TUM_Pose(output_path_each[i], pose_opt_each_vec[i]);
    }
    ros::spinOnce();    

    return 0;
}