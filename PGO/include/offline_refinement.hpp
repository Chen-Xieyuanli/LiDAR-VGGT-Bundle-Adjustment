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
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>  // Include for TF broadcasting
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <malloc.h>

// // ROS related
// ros::Publisher pub_path_orig, pub_path_opt, pub_map_orig, pub_map_opt, pub_plane;
// ros::Subscriber sub_odom, sub_cloud;
// ros::Timer exec_timer;

// // Storage
// IMUST latest_pose;
// IMUST key_pose;
// pcl::PointCloud<PointType>::Ptr key_pcl;
// std::vector<IMUST> poses_vec;
// std::vector<IMUST> poses_vec_opt;
// std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec;
// std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec_opt;

// int start_index = 0;
// int input_counter = 0;
// int slide_size;
// int keyframe_size;
// std::mutex mtx;

// void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
// void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
// void timer_callback(const ros::TimerEvent &event);

// void publish_pointcloud(const pcl::PointCloud<PointType> &cloud, const ros::Publisher &pub);

// void visualize(const vector<IMUST> &poses, const vector<pcl::PointCloud<PointType>::Ptr> &pcls, const ros::Publisher &pub_pos, const ros::Publisher &pub_pcl);

// void downsample_voxel(pcl::PointCloud<PointType>& pc, double voxel_size)
// {
// 	if (voxel_size < 0.01)
// 		return;

// 	std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
// 	size_t pt_size = pc.size();

// 	for (size_t i = 0; i < pt_size; i++)
// 	{
// 		PointType &pt_trans = pc[i];
// 		float loc_xyz[3];
// 		for (int j = 0; j < 3; j++)
// 		{
// 			loc_xyz[j] = pt_trans.data[j] / voxel_size;
// 			if (loc_xyz[j] < 0)
// 				loc_xyz[j] -= 1.0;
// 		}

// 		VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
// 		auto iter = feature_map.find(position);
// 		if (iter != feature_map.end())
// 		{
// 			iter->second.xyz[0] += pt_trans.x;
// 			iter->second.xyz[1] += pt_trans.y;
// 			iter->second.xyz[2] += pt_trans.z;
// 			iter->second.count++;
// 		}
// 		else
// 		{
// 			M_POINT anp;
// 			anp.xyz[0] = pt_trans.x;
// 			anp.xyz[1] = pt_trans.y;
// 			anp.xyz[2] = pt_trans.z;
// 			anp.count = 1;
// 			feature_map[position] = anp;
// 		}
// 	}

// 	pt_size = feature_map.size();
// 	pc.clear();
// 	pc.resize(pt_size);

// 	size_t i = 0;
// 	for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
// 	{
// 		pc[i].x = iter->second.xyz[0] / iter->second.count;
// 		pc[i].y = iter->second.xyz[1] / iter->second.count;
// 		pc[i].z = iter->second.xyz[2] / iter->second.count;
// 		i++;
// 	}
// }