#include "offline_refinement.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;
using namespace mypcl;
typedef pcl::PointXYZINormal PointType;


ros::Publisher pub_path_orig, pub_path_opt, pub_map_orig, pub_map_opt, pub_plane;


void assign_qt(Eigen::Quaterniond& q, Eigen::Vector3d& t,
               const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_)
{
  q.w() = q_.w(); q.x() = q_.x(); q.y() = q_.y(); q.z() = q_.z();
  t(0) = t_(0); t(1) = t_(1); t(2) = t_(2);
}

void read_loop(const string &data_path, std::vector<int> &end_idx, std::vector<int> &start_idx, std::vector<gtsam::Pose3> &T_start_end, std::vector<double> &overlap, std::vector<double> &score){
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
        double temp_overlap, temp_score;
        std::vector<double> matrix_data(16);

        // 读取 end_idx 和 start_idx
        stream >> temp_end_idx >> temp_start_idx;
        end_idx.push_back(temp_end_idx);
        start_idx.push_back(temp_start_idx);

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

void pose_graph_optimization(vector<pose> vec_pose, gtsam::Pose3 T_start_end, string data_path, gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph)
{
    // 设置先验置信度
    gtsam::Vector Prior_Vector6(6);
    Prior_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Prior_Vector6);
    // 设置帧间估计置信度
    gtsam::Vector odom_Vector6(6);
    odom_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr odomModel = gtsam::noiseModel::Diagonal::Variances(odom_Vector6);
    // 设置回环置信度
    gtsam::Vector loopClosure_Vector6(6);
    loopClosure_Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureModel = gtsam::noiseModel::Diagonal::Variances(loopClosure_Vector6);

    for(uint i = 0; i < vec_pose.size(); i++)
    {
        initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));

        if(i == 0)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()),gtsam::Point3(vec_pose[i].t)), priorModel));   // 固定第一帧位置不优化
            continue;  // 遍历到第一帧时不需要加约束
        }

        // 计算两帧之间的相对位姿
        gtsam::Pose3 newPose(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t));
        gtsam::Pose3 oldPose(gtsam::Rot3(vec_pose[i-1].q.toRotationMatrix()), gtsam::Point3(vec_pose[i-1].t));
        gtsam::Pose3 T_new_old = oldPose.between(newPose);

        // 添加因子到因子图
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, T_new_old, odomModel));
        graph.push_back(factor);
    }

    // 加入回环因子（T_start_end是用gicp算出来的）
    gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(vec_pose.size()-1, 0, T_start_end, loopClosureModel));
    graph.push_back(factor);

    // 设置 ISAM2 参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;

    // 创建 ISAM2 优化器并进行更新
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    // 计算最终优化结果
    gtsam::Values results = isam.calculateEstimate();

    cout << "vertex size " << results.size() << endl;

    // 将优化后的位姿保存到 vec_pose_opt 中
    vector<pose> vec_pose_opt;
    vec_pose_opt.resize(vec_pose.size());
    for(uint i = 0; i < results.size(); i++)
    {
        gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
        assign_qt(vec_pose_opt[i].q, vec_pose_opt[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }

    // 保存优化后的位姿
    mypcl::write_pose(vec_pose_opt, data_path, -1);
    printf("pgo complete\n");
}

// Function to create a 4x4 transformation matrix from translation and quaternion
Eigen::Matrix4d createTransformationMatrix(const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;
    return transformation;
}

void publish_pointcloud(const pcl::PointCloud<PointType> &cloud, const ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

void visualize(const vector<IMUST> &poses, const vector<pcl::PointCloud<PointType>::Ptr> &pcls, const ros::Publisher &pub_pos, const ros::Publisher &pub_pcl)
{
    pcl::PointCloud<PointType> pl_send, pl_path;
    int winsize = poses.size();
    for (int i = 0; i < winsize; i++)
    {
        pcl::PointCloud<PointType> pl_tem = *pcls[i];
        down_sampling_voxel(pl_tem, 0.2);
        pl_transform(pl_tem, poses[i]);
        pl_send += pl_tem;
        PointType ap;
        ap.x = poses[i].p.x();
        ap.y = poses[i].p.y();
        ap.z = poses[i].p.z();
        ap.curvature = i;
        pl_path.push_back(ap);
    }

    publish_pointcloud(pl_send, pub_pcl);
    publish_pointcloud(pl_path, pub_pos);
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "offline_refinement");
    ros::NodeHandle nh("~");

    ///////////////////////////////
    //////////// 基础变量定义
    ///////////////////////////////
    int stack_size, keyframe_size;
    std::string data_path;
    double leaf_size,BAradius;
    bool if_visualize = false;

    std::vector<IMUST> poses_vec_opt;
    std::vector<IMUST> poses_vec_ori;
    std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec_opt;   // 这个其实是空的 因为只优化位姿
    std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec_ori; 
    std::vector<mypcl::pose> pose_vec; // 用来读取 json中的数据
    std::string filename;   // json文件名

    nh.getParam("if_visualize", if_visualize);
    nh.getParam("data_path", data_path);
    nh.getParam("stack_size", stack_size);
    nh.getParam("leaf_size", leaf_size);

    nh.param<double>("voxel_size", voxel_size, 2);
    nh.param<int>("min_plane_size", min_plane_size, 20);
    nh.param<double>("BAradius", BAradius, 2);
    nh.param<int>("keyframe_size", keyframe_size, 5);

    ros::Publisher json_index_pub = nh.advertise<std_msgs::Int32>("/json_index", 10);
    pub_path_orig = nh.advertise<sensor_msgs::PointCloud2>("/path_orig", 100);
    pub_path_opt = nh.advertise<sensor_msgs::PointCloud2>("/path_opt", 100);
    pub_map_orig = nh.advertise<sensor_msgs::PointCloud2>("/map_orig", 100);
    pub_map_opt = nh.advertise<sensor_msgs::PointCloud2>("/map_opt", 100);
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("/plane", 100);

    /***************************************************  回环约束因子图优化   *************************************************************************/
    ///////////////////////////////
    //////////// 使用PGO处理回环约束
    ///////////////////////////////

    /******************************************************读入回环约束**************************************************************************************/
    // string loop_path = "/home/jack/FastLab/Multi-UAV-Map-Post-Process/data/loop.txt";
    // vector<int> end_idx;
    // vector<int> start_idx;  
    // vector<gtsam::Pose3> T_loop;
    // vector<double> overlap;
    // vector<double> score;
    // read_loop(loop_path, end_idx, start_idx, T_loop, overlap, score);

    /**************  选择适当的回环约束  ****************/
    
    // Load pose vector
    filename = data_path + "pose.json";
    pose_vec = mypcl::read_pose(filename);
    size_t pose_size = pose_vec.size();
    std::cout << "[Debug] pose_vec loaded! => Size: " << pose_size << std::endl;
    
    // Initialize point cloud pointers
    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr start_frame_stack(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr end_frame_stack(new pcl::PointCloud<PointType>);

    // 假设头尾帧就是回环发生的位置
    for (size_t i = 0; i < stack_size; i++) {
        // Stack to start_frame_stack
        pc_tmp->clear();
        mypcl::loadPCD(data_path + "pcd/", pc_tmp, i);
        mypcl::transform_pointcloud(*pc_tmp, *pc_tmp, pose_vec[i].t, pose_vec[i].q);
        *start_frame_stack += *pc_tmp;

        // Stack to end_frame_stack
        pc_tmp->clear();
        int index = pose_size - 1 - i;
        mypcl::loadPCD(data_path + "pcd/", pc_tmp, index);
        mypcl::transform_pointcloud(*pc_tmp, *pc_tmp, pose_vec[index].t, pose_vec[index].q);
        *end_frame_stack += *pc_tmp;
    }

    down_sampling_voxel(*start_frame_stack, leaf_size);
    down_sampling_voxel(*end_frame_stack, leaf_size);

    // Transform to local coordinate frames
    Eigen::Matrix4d start_transform = createTransformationMatrix(pose_vec[0].t, pose_vec[0].q).inverse();
    Eigen::Matrix4d end_transform = createTransformationMatrix(pose_vec[pose_size - 1].t, pose_vec[pose_size - 1].q).inverse();

    pcl::transformPointCloud(*start_frame_stack, *start_frame_stack, start_transform);
    pcl::transformPointCloud(*end_frame_stack, *end_frame_stack, end_transform);

    // Perform GICP alignment to get rotation and translation
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(start_frame_stack);
    gicp.setInputTarget(end_frame_stack);

    pcl::PointCloud<PointType> aligned;
    gicp.align(aligned);

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    if (gicp.hasConverged()) {
        std::cout << "GICP converged." << std::endl;

        // Get the transformation matrix
        Eigen::Matrix4d transformation = gicp.getFinalTransformation().cast<double>();
        rotation = transformation.block<3, 3>(0, 0);
        translation = transformation.block<3, 1>(0, 3);
        // std::cout << "Rotation:\n" << rotation << std::endl;
        // std::cout << "Translation:\n" << translation.transpose() << std::endl;
    } else {
        std::cout << "GICP did not converge. Exit!" << std::endl;
        return 0;
    }

    gtsam::Pose3 T_start_end = gtsam::Pose3(gtsam::Rot3(rotation),gtsam::Point3(translation));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    pose_graph_optimization(pose_vec, T_start_end, data_path, initial, graph);
    // 发布json的index，用于另一节点的可视化
    std_msgs::Int32 indexmsg; // 创建消息对象
    indexmsg.data = 0;  // 默认第一次PGO后是pose_0.json
    json_index_pub.publish(indexmsg); // 发布index给visualization

    /***************************************************  局部BALM2优化   *************************************************************************/


    ///////////////////////////////
    //////////// BALM 优化
    ///////////////////////////////
    filename = data_path + "pose_0.json";
    pose_vec = mypcl::read_pose(filename);
    // 现在要使用kdtree,来把所有的位姿加载进来,我需要表示的维度是 x,y,z and index
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < pose_size; i++)
    {
        int index = i;
        Eigen::Vector3d xyz = pose_vec[index].t;
        pcl::PointXYZI point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        point.intensity = static_cast<float>(index);  // 用索引作为强度值
        cloud->points.push_back(point);
    }

    // 3. 构建 KDTree
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    // 4. 查询最近邻（模拟回环位置是0,0）
    pcl::PointXYZI search_point;
    search_point.x = 0.0f; // 查询点的 x 坐标
    search_point.y = 0.0f; // 查询点的 y 坐标
    search_point.z = 0.0f; // 查询点的 z 坐标

    // 5. 半径查询
    float radius = BAradius; // 半径
    std::vector<int> point_idx_radius_search;          // 半径内点索引
    std::vector<float> point_radius_squared_distance;  // 半径内距离平方

    poses_vec_ori.clear();
    pcl_vec_ori.clear();
    if (kdtree.radiusSearch(search_point, radius, point_idx_radius_search, point_radius_squared_distance) > 0)
    {
    // 6. 搜索到最近点后，按照index排序，顺序的进行聚合，非顺序的直接丢弃。这里需要规定最大聚合数量，目前没去做 也感觉不一定有必要，也可以跳帧去做（keyframe_size）
        win_size = point_idx_radius_search.size();
        std::cout<<"***************win_size:"<<win_size<<std::endl;
        for (size_t i = 0; i < win_size; i++)
        {
            int index = point_idx_radius_search[i]; // 因为是按照json文件顺序存储的，所以在里的index是可以直接拿来作为pose_vec的索引的。
            if (index!= static_cast<int>(cloud->points.at(index).intensity))
            {
                ROS_WARN("kd-tree order is not aligned with pose_vec order, return!");
                return 0;
            }
            // std::cout<<"index_r:"<<index<<"  intensity:"<<cloud->points.at(index).intensity<<std::endl;
            IMUST pose;
            pose.p = pose_vec[index].t;
            pose.R = pose_vec[index].q.toRotationMatrix();
            pose.t = 0;
            poses_vec_ori.push_back(pose);
            pcl::PointCloud<PointType>::Ptr pc_tmpp(new pcl::PointCloud<PointType>);
            mypcl::loadPCD(data_path + "pcd/", pc_tmpp, index);
            pcl_vec_ori.push_back(pc_tmpp);
            std::cout<<"loadPCD:"<<index<<std::endl;
        }
    }else
    {
        std::cout<<"No ajacent keyframe found! return!"<<std::endl;
        return 0;
    }
    std::cout<<"***************read done!"<<std::endl;

    visualize(poses_vec_ori, pcl_vec_ori, pub_path_orig, pub_map_orig);

    // Normalize the data
    IMUST es0 = poses_vec_ori[0];
    for (uint i = 0; i < poses_vec_ori.size(); i++)
    {
        poses_vec_ori[i].p = es0.R.transpose() * (poses_vec_ori[i].p - es0.p);
        poses_vec_ori[i].R = es0.R.transpose() * poses_vec_ori[i].R;
    }

    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;

    eigen_value_array[0] = 1.0 / 16;
    eigen_value_array[1] = 1.0 / 16;
    eigen_value_array[2] = 1.0 / 9;

    for (int i = 0; i < win_size; i++)
    {
        cut_voxel(surf_map, *pcl_vec_ori[i], poses_vec_ori[i], i);
    }

    pcl::PointCloud<PointType> pcl_plane;
    VOX_HESS voxhess;
    int counttt=0;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
        iter->second->recut(win_size);
        iter->second->tras_opt(voxhess, win_size);
        iter->second->tras_display(pcl_plane, win_size);
        std::cout<<"tras_opt"<<counttt++<<std::endl;
    }

    publish_pointcloud(pcl_plane, pub_plane);

    std::cout<<"////////////////////////////////"<<std::endl;
    std::cout<<"voxhess.plvec_voxels.size():"<<voxhess.plvec_voxels.size()<<std::endl;
    std::cout<<"3 * poses_vec_ori.size():"<<3 * poses_vec_ori.size()<<std::endl;

    // if (voxhess.plvec_voxels.size() < 3 * poses_vec_ori.size())
    // {
    //     printf("Initial error too large.\n");
    //     printf("Please loose plane determination criteria for more planes.\n");
    //     printf("The optimization is skipped.\n");
    //     return 0;
    // }

    // Optimize the poses
    BALM2 opt_lsv;
    // opt_lsv.damping_iter(poses_vec_ori, voxhess);
    opt_lsv.damping_iter_no_odom_constraints(poses_vec_ori, voxhess);

    for (auto iter = surf_map.begin(); iter != surf_map.end();)
    {
        delete iter->second;
        surf_map.erase(iter++);
    }
    surf_map.clear();
    malloc_trim(0);

    // Denormalize the data
    poses_vec_opt.resize(win_size);
    for (uint i = 0; i < poses_vec_ori.size(); i++)
    {
        poses_vec_opt[i].p = es0.R * poses_vec_ori[i].p + es0.p;
        poses_vec_opt[i].R = es0.R * poses_vec_ori[i].R;
    }

    visualize(poses_vec_opt, pcl_vec_ori, pub_path_opt, pub_map_opt);   // opt后的pcl与ori没区别，所以直接用ori。
    sleep(0.5);
    // 我现在此前已经通过odom建立起了initial和graph，并将初末位置的回环约束作为因子加入到了PGO中。
    // 接下来，我又做了BA，得到了在回环发生的位置附近，多个关键帧的pose，存储到了poses_vec_opt中。
    // 这些参与优化的关键帧index记录在point_idx_radius_search中，可以用于索引。
    // 我下面这个步骤想做的是，将参与BA的关键帧索引的当前poses_vec_opt更新到对应的intial中的prior，
    // 以及，将BA得到的新poses，按照顺序组成双边因子替换/新增到graph中。这里，我所说的顺序是值的index顺序必须相邻。
    // 比如,1和2,2和3组成的因子，如果因子不相邻，比如，point_idx_radius_search经过从小到大排序之后，为0,1,2,3,4,8,9,10,12,13,14,17,20,21,22
    // 那么，我们可以组成：0-1,1-2,2-3,3-4,8-9,9-10,12-13,20-21,21-22（注意，这里17因为是单独的一个，所以就不处理了，直接不把他加入因子图）
    // 这些因子正常情况下应该是存在于graph中的，只需要用BA得到的poses_vec_opt位姿，计算一下translation，更新到factor中去即可（你在更新进去的时候可以先查找是否存在这个factor，存在就更新model和translation，不存在直接创建）
    // 另外，关于Model，因为我认为BA是最准的，所以它的model的置信度应该足够高，我觉得约束的model的置信度应该是BA>loopclosure>odom（odom来自前端LIO直接出的）
    
    
    /***************************************************  全局因子图优化   *************************************************************************/
    // 更新约束后，全局的因子图优化
    // 更新PGO


    // 输出优化后的结果
    std::cout << "[PGO] Optimization results after BA:" << std::endl;
    for (size_t i = 0; i < results.size(); ++i) {
        gtsam::Pose3 optimized_pose = results.at<gtsam::Pose3>(i);
        std::cout << "Pose " << i << ": " << optimized_pose << std::endl;
    }

    return 0;
}