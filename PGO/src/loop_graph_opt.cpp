#include "utils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Core>
using namespace std;

void pose_graph_optimization(vector<pose> vec_pose, 
                             std::vector<gtsam::Pose3> T_start_end, 
                             std::vector<std::pair<int, int>> loop_idx, 
                             gtsam::Values &initial, 
                             gtsam::NonlinearFactorGraph &graph, 
                             vector<pose> &pose_opt,
                             std::vector<int> pose_size_each,
                             std::vector<int> loop_type) 
{
    // 设置先验置信度
    gtsam::Vector Prior_Vector6(6);
    // Prior_Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    Prior_Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Prior_Vector6);
    
    // 设置帧间估计置信度
    gtsam::Vector odom_Vector6(6);
    odom_Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;

    // odom_Vector6 << 0, 0, 0, 0, 0, 0;
    gtsam::noiseModel::Diagonal::shared_ptr odomModel = gtsam::noiseModel::Diagonal::Variances(odom_Vector6);
    
    // 设置回环置信度  inter loop
    gtsam::Vector loopClosure_Vector6(6);
    // loopClosure_Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    loopClosure_Vector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureModel = gtsam::noiseModel::Diagonal::Variances(loopClosure_Vector6);

    // 设置回环置信度  self loop
    gtsam::Vector loopClosure_self(6);
    loopClosure_self << 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
    gtsam::noiseModel::Diagonal::shared_ptr loopSelfModel = gtsam::noiseModel::Diagonal::Variances(loopClosure_self);
    

    // 两个机器人之间的弱连接
    gtsam::Vector inter_Vector6(6);
    inter_Vector6 << 1, 1, 1, 1, 1, 1;
    gtsam::noiseModel::Diagonal::shared_ptr interModel = gtsam::noiseModel::Diagonal::Variances(inter_Vector6);

    std::vector<int> step_vector;
    int len = 0;
    for(int i = 0; i < pose_size_each.size(); i++){
        len += pose_size_each[i];
        step_vector.push_back(len);
    }
    for(int i = 0; i < step_vector.size(); i++){
        std::cout << "step_vector " << i << " " << step_vector[i] << std::endl;
    }

    
    std::cout << "pose_size_each[0] " << pose_size_each[0] <<std::endl;
    std::cout << "pose_size_each[1] " << pose_size_each[1] <<std::endl;

    for (uint i = 0; i < vec_pose.size(); i++)
    {   
        bool addFlag = true;
        //第一个机器人第一个位姿
        if (i == 0) 
        {
            // 插入当前位姿到初始估计中
            if (!initial.exists(i)) {
                initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));
            }

            // 添加先验因子
            std::cout << "current pose index: " << i << " current i: " << i << std::endl;
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)), priorModel));
            continue;
        }

        // 其他位姿
        if (!initial.exists(i)) {
            initial.insert(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)));
        }

        // 计算相邻帧之间的相对位姿
        gtsam::Pose3 newPose(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t));
        gtsam::Pose3 oldPose(gtsam::Rot3(vec_pose[i-1].q.toRotationMatrix()), gtsam::Point3(vec_pose[i-1].t));
        gtsam::Pose3 T_new_old = oldPose.between(newPose);

        //判断是否添加
        for(int k = 0; k < step_vector.size(); k++){
            if(i == step_vector[k]){
                addFlag = false;
            }
        }

        if(addFlag){
            // 添加帧间约束
            graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, T_new_old, odomModel));
            
        }
        else{
            // graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, T_new_old, interModel));
            // graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(vec_pose[i].q.toRotationMatrix()), gtsam::Point3(vec_pose[i].t)), interModel));
            std::cout << "i " << i << std::endl;
            std::cout << "no constraints added between " << i << " and " << i -1 << std::endl;
        }

    }

    // 添加回环因子
    for (size_t i = 0; i < loop_idx.size(); ++i)
    {
        int start_idx = loop_idx[i].first;
        int end_idx = loop_idx[i].second;
        
        // 获取回环起始和结束的里程计位姿
        gtsam::Pose3 Pose_start_world = initial.at<gtsam::Pose3>(start_idx);  // 回环起始帧在世界坐标系中的位姿
        gtsam::Pose3 Pose_end_world = initial.at<gtsam::Pose3>(end_idx);  // 回环结束帧在世界坐标系中的位姿

        // 计算从回环起始到结束的位姿差异
        gtsam::Pose3 T_start_end_odom = Pose_start_world.inverse() * Pose_end_world;

        // 获取ICP计算得到的回环相对位姿变换
        gtsam::Pose3 T_icp = T_start_end[i];  // ICP计算的相对位姿

        gtsam::Pose3 true_loop_transform = T_icp;
        
        // 添加回环因子
        if(loop_type[i] == 0){
            graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(start_idx, end_idx, true_loop_transform, loopSelfModel));
            std::cout << "adding self loop!!!!!!!!!!!" << std::endl;
        }

        else{
            graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(start_idx, end_idx, true_loop_transform, loopClosureModel));
            std::cout << "adding inter loop!!!!!!!!!!!" << std::endl;
        }

        
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

    cout << "vertex size " << results.size() << endl;

    // 保存优化结果
    pose_opt.resize(vec_pose.size());
    for (uint i = 0; i < results.size(); i++)
    {
        gtsam::Pose3 pose = results.at<gtsam::Pose3>(i);
        assign_qt(pose_opt[i].q, pose_opt[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }
    printf("Multi-Robot Factor Graph Optimization Complete!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "loop_graph_opt");
    ros::NodeHandle nh("~");
    /***************    read poses ---> read loopclosures ---> Factor Graph Optimization with Loop Constraints *********************/

    //点云pcd和位姿pose的路径
    std::string data_path;
    nh.getParam("data_path", data_path);

    //回环loop.txt的路径
    std::string loop_path;
    nh.getParam("loop_path", loop_path);

    std::string origin_pose_name;
    nh.getParam("origin_pose_name", origin_pose_name);

    //机器人数量
    int robot_num;
    nh.getParam("robot_num", robot_num);

    // pose vector，多机器人
    std::vector<pose> pose_vecs[robot_num];    
    std::vector<int> pose_size_each(robot_num);
    int pose_total = 0;  //所有机器人位姿数量之和
    std::vector<pose> pose_vec_total;

    //多机器人回环数据
    std::vector<std::pair<int, int>> end_idx;
    std::vector<std::pair<int, int>> start_idx;
    std::vector<gtsam::Pose3> T_start_end;
    std::vector<double> overlap;
    std::vector<double> score;
    std::vector<int> loop_type;   //wlj add 0 for self 1 for inter
    int loop_num;

    //读取每个机器人的pose
    for (int i = 0; i < robot_num; i++)
    {
        std::vector<pose> pose_vec;
        int pose_size = read_json_pose_new(pose_vec, data_path + std::to_string(i) + "/", i, origin_pose_name);
        pose_vecs[i] = pose_vec;
        pose_size_each[i] = pose_size;
        pose_total += pose_size;
        pose_vec_total.insert(pose_vec_total.end(), pose_vec.begin(), pose_vec.end());
    }

    std::cout << "Total pose size: " << pose_vec_total.size() << std::endl;

    //读取回环的loop.txt
    read_multi_loop(loop_path, end_idx, start_idx, T_start_end, overlap, score);
    loop_num = end_idx.size();
    std::cout << "Loop num: " << end_idx.size() << std::endl;
    std::vector<std::pair<int, int>> loop_idx;
    for(int i = 0; i < loop_num; i++)
    {
        int start_drone_id = start_idx[i].first;
        int end_drone_id = end_idx[i].first;
        int start_loop_idx = 0;
        int end_loop_idx = 0;
        if(start_drone_id == end_drone_id){
            loop_type.push_back(0);
        }
        else{
            loop_type.push_back(1);
        }
        for(int j = 0; j < robot_num; j++)
        {
            if(j == start_drone_id)
            {
                start_loop_idx += start_idx[i].second;
                break;
            }
            else
            {
                start_loop_idx += pose_size_each[j];
            }
        }
        for(int j = 0; j < robot_num; j++)
        {
            if(j == end_drone_id)
            {
                end_loop_idx += end_idx[i].second;
                break;
            }
            else
            {
                end_loop_idx += pose_size_each[j];
            }
        }
        loop_idx.push_back(std::make_pair(start_loop_idx, end_loop_idx));
    }
    // for(int i = 0; i < loop_num; i++)
    // {
    //     std::cout << "end_idx: " << end_idx[i].first << " " << end_idx[i].second << std::endl;
    //     std::cout << "start_idx: " << start_idx[i].first << " " << start_idx[i].second << std::endl;
    //     std::cout << "loop_idx: " << loop_idx[i].first << " " << loop_idx[i].second << std::endl;
    //     std::cout << "T_start_end: " << T_start_end[i] << std::endl;
    // }
    

    //可视化 RViz ，优化前后的点云和位姿
    ros::Publisher pose_pub_initial = nh.advertise<visualization_msgs::MarkerArray>("/poses_initial", 100);
    ros::Publisher pcl_pub_initial = nh.advertise<sensor_msgs::PointCloud2>("/loop_map_initial", 100);
    ros::Publisher pose_pub_opt = nh.advertise<visualization_msgs::MarkerArray>("/poses_opt", 100);
    ros::Publisher pcl_pub_opt = nh.advertise<sensor_msgs::PointCloud2>("/loop_map_opt", 100);

    //可视化颜色
    std_msgs::ColorRGBA green = createColorRGBA(0.0, 1.0, 0.0, 1.0);
    std_msgs::ColorRGBA red = createColorRGBA(1.0, 0.0, 0.0, 1.0);
    std_msgs::ColorRGBA blue = createColorRGBA(0.0, 0.0, 1.0, 1.0);

    //因子图优化
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    //输出优化后的位姿态
    std::string pose_opt_path;
    nh.getParam("pose_opt_path", pose_opt_path);
    std::string pose_opt_each_path[robot_num];
    for(int i = 0; i < robot_num; i++)
    {
        pose_opt_each_path[i] = pose_opt_path + std::to_string(i) + "/";
    }

    //多机的因子图优化
    std::vector<pose> pose_opt_total;
    std::vector<pose> pose_opt_each_vec[robot_num];
    pose_graph_optimization(pose_vec_total, T_start_end, loop_idx,  initial, graph, pose_opt_total, pose_size_each, loop_type);
    int pose_index = 0;
    for(int i = 0; i < robot_num; i++)
    {
        pose_opt_each_vec[i].insert(pose_opt_each_vec[i].end(), pose_opt_total.begin() + pose_index, pose_opt_total.begin() + pose_index + pose_size_each[i]);
        pose_index += pose_size_each[i];
    }
    pose_opt_total.clear();
    //保存优化后的位姿
    for(int i = 0; i < robot_num; i++)
    {   
        std::cout << "Writing" << i <<  " to : " << pose_opt_each_path[i] << std::endl;
        write_pose(pose_opt_each_vec[i], pose_opt_each_path[i], -1);
    }
    ros::spinOnce();    

    return 0;
}