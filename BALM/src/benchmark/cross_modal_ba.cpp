#include "tools.hpp"
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <ctime>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "bavoxel.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <malloc.h>

#include <iomanip>
// #include <execution>
#include <omp.h>

using namespace std;

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = ros::Time::now();
  pub.publish(output);
}

ros::Publisher pub_path, pub_test, pub_show, pub_cute;
// Jack add
ros::Publisher pcd_pub, pose_pub;

int read_pose(vector<double> &tims, PLM(3) &rots, PLV(3) &poss, string prename)
{
  string readname = prename + "alidarPose.csv";

  cout << readname << endl;
  ifstream inFile(readname);

  if(!inFile.is_open())
  {
    printf("open fail\n"); return 0;
  }

  int pose_size = 0;
  string lineStr, str;
  Eigen::Matrix4d aff;
  vector<double> nums;

  int ord = 0;
  while(getline(inFile, lineStr))
  {
    ord++;
    stringstream ss(lineStr);
    while(getline(ss, str, ','))
      nums.push_back(stod(str));

    if(ord == 4)
    {
      for(int j=0; j<16; j++)
        aff(j) = nums[j];

      Eigen::Matrix4d affT = aff.transpose();

      rots.push_back(affT.block<3, 3>(0, 0));
      poss.push_back(affT.block<3, 1>(0, 3));
      tims.push_back(affT(3, 3));
      nums.clear();
      ord = 0;
      pose_size++;
    }
  }

  return pose_size;
}

void read_file(vector<IMUST> &x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls, string &prename)
{
  prename = prename + "/datas/benchmark_realworld/";

  PLV(3) poss; PLM(3) rots;
  vector<double> tims;
  int pose_size = read_pose(tims, rots, poss, prename);
  
  for(int m=0; m<pose_size; m++)
  {
    string filename = prename + "full" + to_string(m) + ".pcd";

    pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
    pcl::PointCloud<pcl::PointXYZI> pl_tem;
    pcl::io::loadPCDFile(filename, pl_tem);
    for(pcl::PointXYZI &pp: pl_tem.points)
    {
      PointType ap;
      ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
      ap.intensity = pp.intensity;
      pl_ptr->push_back(ap);
    }

    pl_fulls.push_back(pl_ptr);

    IMUST curr;
    curr.R = rots[m]; curr.p = poss[m]; curr.t = tims[m];
    x_buf.push_back(curr);
  }
  

}

void data_show(vector<IMUST> x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls)
{
  IMUST es0 = x_buf[0];
  for(uint i=0; i<x_buf.size(); i++)
  {
    x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
    x_buf[i].R = es0.R.transpose() * x_buf[i].R;
  }

  pcl::PointCloud<PointType> pl_send, pl_path;
  int winsize = x_buf.size();
  for(int i=0; i<winsize; i++)
  {
    pcl::PointCloud<PointType> pl_tem = *pl_fulls[i];
    down_sampling_voxel(pl_tem, 0.05);
    pl_transform(pl_tem, x_buf[i]);
    pl_send += pl_tem;

    if((i%200==0 && i!=0) || i == winsize-1)
    {
      pub_pl_func(pl_send, pub_show);
      pl_send.clear();
      sleep(0.5);
    }

    PointType ap;
    ap.x = x_buf[i].p.x();
    ap.y = x_buf[i].p.y();
    ap.z = x_buf[i].p.z();
    ap.curvature = i;
    pl_path.push_back(ap);
  }

  pub_pl_func(pl_path, pub_path);
}

// 发布不同颜色的相机 lidar点云
void data_show_new(std::vector<IMUST> &x_buf, 
                   std::vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls)
{
  if(x_buf.empty() || pl_fulls.empty()) return;

  IMUST es0 = x_buf[0];
  // 坐标归一化到第一个位姿
  for(size_t i=0; i<x_buf.size(); i++)
  {
    x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
    x_buf[i].R = es0.R.transpose() * x_buf[i].R;
  }

  pcl::PointCloud<pcl::PointXYZRGB> pl_send, pl_path;
  int winsize = x_buf.size();

  for(int i=0; i<winsize; i++)
  {
    pcl::PointCloud<PointType> pl_tem = *pl_fulls[i];
    down_sampling_voxel(pl_tem, 0.05);
    pl_transform(pl_tem, x_buf[i]);

    // 转成彩色点云
    pcl::PointCloud<pcl::PointXYZRGB> pl_colored;
    pl_colored.reserve(pl_tem.size());

    uint8_t r = (x_buf[i].type == 0 ? 0   : 255); // LiDAR 蓝色, Camera 红色
    uint8_t g = (x_buf[i].type == 0 ? 0   : 0);
    uint8_t b = (x_buf[i].type == 0 ? 255 : 0);

    for(const auto &pt : pl_tem.points) {
      pcl::PointXYZRGB cp;
      cp.x = pt.x; cp.y = pt.y; cp.z = pt.z;
      cp.r = r; cp.g = g; cp.b = b;
      pl_colored.push_back(cp);
    }

    pl_send += pl_colored;

    if((i%200==0 && i!=0) || i == winsize-1)
    {
      pub_pl_func(pl_send, pcd_pub);
      pl_send.clear();
      ros::Duration(0.5).sleep();
    }

    // 轨迹点
    pcl::PointXYZRGB ap;
    ap.x = x_buf[i].p.x();
    ap.y = x_buf[i].p.y();
    ap.z = x_buf[i].p.z();
    ap.r = r; ap.g = g; ap.b = b;
    pl_path.push_back(ap);
  }

  pub_pl_func(pl_path, pose_pub);
}


// 读取TUM格式的位姿文件
int read_TUM_pose(std::string filename, std::vector<IMUST> &x_buf, int type) {
    std::fstream file;
    file.open(filename);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    while (file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        Eigen::Quaterniond qd(qw, qx, qy, qz);
        Eigen::Vector3d td(tx, ty, tz);
        IMUST curr;
        curr.R = qd.toRotationMatrix();
        curr.p = td;
        curr.t = timestamp;
        curr.type = type;
        x_buf.push_back(curr);
    }
    return x_buf.size();
}

// 读取对应的点云
void read_point_clouds(const std::string &base_path, 
                       int num, 
                       std::vector<pcl::PointCloud<PointType>::Ptr> &point_clouds)
{
    std::cout << "Loading " << num << " pcd files from " << base_path << std::endl;
    for (int i = 0; i < num; ++i)
    {   
        std::string filename = base_path +  std::to_string(i) + ".pcd";
      pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
      pcl::PointCloud<pcl::PointXYZI> pl_tem;
      pcl::io::loadPCDFile(filename, pl_tem);
      for(pcl::PointXYZI &pp: pl_tem.points)
      {
        PointType ap;
        ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
        ap.intensity = 0.0f;
        ap.normal_x = 0.0f;
        ap.normal_y = 0.0f;
        ap.normal_z = 0.0f;
        ap.curvature = 0.0f;
        pl_ptr->push_back(ap);
      }

      point_clouds.push_back(pl_ptr);
    }
}

void read_XYZ_point_clouds(const std::string &base_path, 
                                int num, 
                                std::vector<pcl::PointCloud<PointType>::Ptr> &point_clouds)
{
    std::cout << "Loading " << num << " camera pcd files from " << base_path << std::endl;
    for (int i = 0; i < num; ++i)
    {   
        std::string filename = base_path +  std::to_string(i) + ".pcd";
        pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
        pcl::PointCloud<pcl::PointXYZ> pl_tem;
        pcl::io::loadPCDFile(filename, pl_tem);
        for(pcl::PointXYZ &pp: pl_tem.points)
        {
            PointType ap;
            ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
            ap.intensity = 0.0f;
            ap.normal_x = 0.0f;
            ap.normal_y = 0.0f;
            ap.normal_z = 0.0f;
            ap.curvature = 0.0f;
            pl_ptr->push_back(ap);
        }
        point_clouds.push_back(pl_ptr);
    }
}

// 读取camera的RGB点云 转换成 typedef pcl::PointXYZINormal PointType;
void read_RGB_point_clouds(const std::string &base_path, 
                                int num, 
                                std::vector<pcl::PointCloud<PointType>::Ptr> &point_clouds)
{
    std::cout << "Loading " << num << " camera pcd files from " << base_path << std::endl;
    for (int i = 0; i < num; ++i)
    {   
        std::string filename = base_path +  std::to_string(i) + ".pcd";
        pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
        pcl::PointCloud<pcl::PointXYZRGB> pl_tem;
        pcl::io::loadPCDFile(filename, pl_tem);
        for(pcl::PointXYZRGB &pp: pl_tem.points)
        {
            PointType ap;
            ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
            ap.intensity = 0.0f;
            pl_ptr->push_back(ap);
        }
        point_clouds.push_back(pl_ptr);
    }
}

// chunk内的lidar camera 数据合并，需要合并
void merge_multi_modal_data(
    const std::vector<IMUST> &lidar_pose, 
    const std::vector<IMUST> &camera_pose, 
    const std::vector<pcl::PointCloud<PointType>::Ptr> &lidar_points,
    const std::vector<pcl::PointCloud<PointType>::Ptr> &camera_points,
    std::vector<IMUST> &merged_pose,
    std::vector<pcl::PointCloud<PointType>::Ptr> &merged_points)
{
    // 清空输出
    merged_pose.clear();
    merged_points.clear();

    // 临时结构体绑定 pose 和 pointcloud
    struct PoseCloud {
        IMUST pose;
        pcl::PointCloud<PointType>::Ptr cloud;
    };

    std::vector<PoseCloud> all_data;
    all_data.reserve(lidar_pose.size() + camera_pose.size());

    // lidar
    for (size_t i = 0; i < lidar_pose.size(); ++i) {
        all_data.push_back({lidar_pose[i], lidar_points[i]});
    }

    // camera
    for (size_t i = 0; i < camera_pose.size(); ++i) {
        all_data.push_back({camera_pose[i], camera_points[i]});
    }

    // 按时间戳排序
    std::sort(all_data.begin(), all_data.end(),
              [](const PoseCloud &a, const PoseCloud &b) {
                  return a.pose.t < b.pose.t;
              });

    // 拆开存储
    for (auto &pc : all_data) {
        merged_pose.push_back(pc.pose);
        merged_points.push_back(pc.cloud);
    }
}

//split merged data
void split_multi_modal_data(const std::vector<IMUST> &merged_pose,
                             const std::vector<pcl::PointCloud<PointType>::Ptr> &merged_points,
                             std::vector<IMUST> &lidar_pose,
                             std::vector<IMUST> &camera_pose,
                             std::vector<pcl::PointCloud<PointType>::Ptr> &lidar_points,
                             std::vector<pcl::PointCloud<PointType>::Ptr> &camera_points)
{
    for (size_t i = 0; i < merged_pose.size(); ++i) {
        if (merged_pose[i].type == 0) {
            lidar_pose.push_back(merged_pose[i]);
            lidar_points.push_back(merged_points[i]);
        } else {
            camera_pose.push_back(merged_pose[i]);
            camera_points.push_back(merged_points[i]);
        }
    }
}

//save TUM
void save_TUM_Pose(const std::string &file_path,
               const std::vector<IMUST> &poses)
{
    std::ofstream pose_file(file_path);

    for (const auto &pose : poses) {
      //change R to qx qy qz qw
      Eigen::Quaterniond q(pose.R);
      // change p to x y z
      //高小数精度
      pose_file << std::fixed << std::setprecision(8) << pose.t << " " 
      << std::fixed << std::setprecision(6) << pose.p.x() << " " 
      << std::fixed << std::setprecision(6) << pose.p.y() << " " 
      << std::fixed << std::setprecision(6) << pose.p.z() << " " 
      << std::fixed << std::setprecision(6) << q.x() << " " 
      << std::fixed << std::setprecision(6) << q.y() << " " 
      << std::fixed << std::setprecision(6) << q.z() << " " 
      << std::fixed << std::setprecision(6) << q.w() << std::endl;
    }
    pose_file.close();
}

// merge lidar
pcl::PointCloud<PointType>::Ptr mergeLidarPointsToFirstFrame(
    const std::vector<IMUST>& lidar_pose,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& lidar_points)
{
    if (lidar_pose.empty() || lidar_points.empty()) {
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    // 第一帧位姿
    Eigen::Matrix3d R0 = lidar_pose[0].R;
    Eigen::Vector3d p0 = lidar_pose[0].p;

    pcl::PointCloud<PointType>::Ptr lidar_points_all(new pcl::PointCloud<PointType>());

    for (size_t i = 0; i < lidar_pose.size(); i++) {
        const auto& pose = lidar_pose[i];
        const auto& cloud = lidar_points[i];

        for (const auto& pt : cloud->points) {
            Eigen::Vector3d p_body_i(pt.x, pt.y, pt.z);

            // 转到世界系
            Eigen::Vector3d p_world = pose.R * p_body_i + pose.p;

            // 再转到第 0 帧 body 系
            Eigen::Vector3d p_body0 = R0.transpose() * (p_world - p0);

            PointType new_pt = pt;
            new_pt.x = p_body0.x();
            new_pt.y = p_body0.y();
            new_pt.z = p_body0.z();

            lidar_points_all->points.push_back(new_pt);
        }
    }

    lidar_points_all->width = lidar_points_all->points.size();
    lidar_points_all->height = 1;
    lidar_points_all->is_dense = false;

    return lidar_points_all;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cross_modal_ba");
  ros::NodeHandle n;
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_show = n.advertise<sensor_msgs::PointCloud2>("/map_show", 100);
  pub_cute = n.advertise<sensor_msgs::PointCloud2>("/map_cute", 100);
  pcd_pub  = n.advertise<sensor_msgs::PointCloud2>("/pcd_chunk", 100);
  pose_pub = n.advertise<sensor_msgs::PointCloud2>("/pose_chunk", 100);

  string prename, ofname;
  vector<IMUST> x_buf;
  vector<pcl::PointCloud<PointType>::Ptr> pl_fulls;

  n.param<double>("voxel_size", voxel_size, 1);
  // string file_path;
  // n.param<string>("file_path", file_path, "");

  // Jack add 新变量

  int chunk_num;
  n.getParam("chunk_num", chunk_num);
  string result_path;
  n.getParam("result_path", result_path);
  string data_base_path;
  n.getParam("data_base_path", data_base_path);
  double leaf_size;
  n.getParam("leaf_size", leaf_size);
  // Jack add 循环

  // read_file(x_buf, pl_fulls, file_path);
  for(int i=0; i<chunk_num; i++)
  {
    std::cout << "Processing chunk " << i << ".........." << std::endl;
    x_buf.clear(); pl_fulls.clear();

    //数据路径
    string lio_data_path = data_base_path + "chunk_" + std::to_string(i) + "/lidar/";
    string vggt_data_path = data_base_path + "chunk_" + std::to_string(i) + "/camera/";

    // 位姿和点云读取
    vector<IMUST> lidar_pose, camera_pose;
    vector<pcl::PointCloud<PointType>::Ptr> lidar_points, camera_points;

    // lidar
    std::cout << "Loading lidar pose from " << lio_data_path + "poses.txt" << std::endl;
    int lidar_pose_num = read_TUM_pose(lio_data_path + "poses.txt", lidar_pose, 0);
    std::cout << "lidar pose num: " << lidar_pose_num << std::endl;
    read_XYZ_point_clouds(lio_data_path + "pcd/", lidar_pose_num, lidar_points);
    std::cout << " lidar pcd num: " << lidar_points.size() << std::endl;
    // pcl::PointCloud<PointType>::Ptr lidar_points_all(new pcl::PointCloud<PointType>());
    // lidar_points_all = mergeLidarPointsToFirstFrame(lidar_pose, lidar_points);

    // camera
    std::cout << "Loading camera pose from " << vggt_data_path + "poses.txt" << std::endl;
    int camera_pose_num = read_TUM_pose(vggt_data_path + "poses.txt", camera_pose, 1);
    std::cout << "camera pose num: " << camera_pose_num << std::endl;
    read_RGB_point_clouds(vggt_data_path + "pcd/", camera_pose_num, camera_points);
    std::cout << "camera pcd num: " << camera_points.size() << std::endl;

    // merge
    // merge_multi_modal_data(lidar_pose, camera_pose, lidar_points, camera_points, x_buf, pl_fulls);
    // x_buf.assign(lidar_pose.begin(), lidar_pose.end());
    // // x_buf.push_back(lidar_pose[0]); 
    // x_buf.insert(x_buf.end(), camera_pose.begin(), camera_pose.end());
    // // pl_fulls.push_back(lidar_points_all);
    // pl_fulls.assign(lidar_points.begin(), lidar_points.end());
    // pl_fulls.insert(pl_fulls.end(), camera_points.begin(), camera_points.end());
    x_buf = camera_pose;
    pl_fulls = camera_points;
    // 释放内存
    lidar_pose.clear(); 
    camera_pose.clear();
    lidar_points.clear();
    camera_points.clear();
    //下采样
    std::vector<pcl::PointCloud<PointType>::Ptr> pl_downs;
    for (size_t i = 0; i < pl_fulls.size(); i++) 
    {
      pcl::PointCloud<PointType>::Ptr cloud_down(new pcl::PointCloud<PointType>());
      pcl::VoxelGrid<PointType> sor;
      sor.setInputCloud(pl_fulls[i]);
      sor.setLeafSize(leaf_size, leaf_size, leaf_size);
      sor.filter(*cloud_down);
      pl_downs.push_back(cloud_down);
    }
    pl_fulls = pl_downs;
    pl_downs.clear();

    std::cout << "After merging, total pose num: " << x_buf.size() << std::endl;
    std::cout << "After merging, total pcd num: " << pl_fulls.size() << std::endl;
    // x_buf = lidar_pose;
    // pl_fulls = lidar_points;

    // x_buf = camera_pose;
    // pl_fulls = camera_points;
    //change to first frame
    IMUST es0 = x_buf[0];
    for(uint i=0; i<x_buf.size(); i++)
    {
      x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
      x_buf[i].R = es0.R.transpose() * x_buf[i].R;
    }

    //chunk 大小
    win_size = x_buf.size();
    printf("The size of poses: %d\n", win_size);
    ros::Duration(1.0).sleep();

    data_show(x_buf, pl_fulls);
    // data_show(lidar_pose, lidar_points);

    // data_show_new(x_buf, pl_fulls);
    ros::Duration(6.0).sleep();
    printf("Check the point cloud with the initial poses.\n");
    printf("If no problem, input '1' to continue or '0' to exit...\n");
    // int a; cin >> a; if(a==0) exit(0);

    pcl::PointCloud<PointType> pl_full, pl_surf, pl_path, pl_send;
    for(int iterCount=0; iterCount<1; iterCount++)
    { 
      unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

      eigen_value_array[0] = 1.0 / 16;
      eigen_value_array[1] = 1.0 / 16;
      eigen_value_array[2] = 1.0 / 9;
      std::cout << "before voxelization: " << std::endl;
      for(int i=0; i<win_size; i++)
        cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);
      std::cout << "after voxelization: " << std::endl;
      pcl::PointCloud<PointType> pl_send;
      pub_pl_func(pl_send, pub_show);

      pcl::PointCloud<PointType> pl_cent; pl_send.clear();
      VOX_HESS voxhess;
      for(auto iter=surf_map.begin(); iter!=surf_map.end() && n.ok(); iter++)
      {
        iter->second->recut(win_size);
        iter->second->tras_opt(voxhess, win_size);
        iter->second->tras_display(pl_send, win_size);
      }

      pub_pl_func(pl_send, pub_cute);
      printf("\nThe planes (point association) cut by adaptive voxelization.\n");
      printf("If the planes are too few, the optimization will be degenerated and fail.\n");
      printf("If no problem, input '1' to continue or '0' to exit...\n");
      // int a; cin >> a; if(a==0) exit(0);
      pl_send.clear(); pub_pl_func(pl_send, pub_cute);

      if(voxhess.plvec_voxels.size() < 3 * x_buf.size())
      {
        printf("Initial error too large.\n");
        printf("Please loose plane determination criteria for more planes.\n");
        printf("The optimization is terminated.\n");
        exit(0);
      }
      std::cout << "surf_map size before clear: " << surf_map.size() << std::endl;

      BALM2 opt_lsv;
      std::cout << "before optimization: " << std::endl;
      opt_lsv.damping_iter(x_buf, voxhess);
      std::cout << "after optimization: " << std::endl;

      // for(auto iter=surf_map.begin(); iter!=surf_map.end();)
      // {
      //   delete iter->second;
      //   surf_map.erase(iter++);
      // }
      // surf_map.clear();
      // for (auto &kv : surf_map) {
      //     delete kv.second;
      // }
            // for (size_t i = 0; i < surf_map.size(); i++) {
      //     auto iter = std::next(surf_map.begin(), i);
      //     delete iter->second;
      //     std::cout << "deleting node " << i << std::endl;
      // }
      int count = 0;
      int total = surf_map.size();
      // #pragma omp parallel for
      // for(auto iter=surf_map.begin(); iter!=surf_map.end();)
      // {
      //   delete iter->second;
      //   surf_map.erase(iter++);
      //   std::cout << "deleting node " << count++ << std::endl;
      // }
      #pragma omp parallel for
      for (int i = 0; i < total; i++) {
          auto iter = surf_map.begin();
          std::advance(iter, i);

          delete iter->second;

          #pragma omp critical
          {
              count++;
              float progress = (float)count / total * 100.0f;
              std::cout << "\rDeleting nodes: " 
                        << std::setw(4) << (int)progress << "% (" 
                        << count << "/" << total << ")" 
                        << std::flush;
          }
      }

    std::cout << std::endl; // 最后换行
      std::cout << "after deleting: " << std::endl;
      surf_map.clear();
      std::cout << "after clearing: " << std::endl;

      malloc_trim(0);

    }
    //Jack add Denormalization
    std::cout << "Denormalization.........." << std::endl;
    std::vector<IMUST> x_buf_opt;
    x_buf_opt.resize(x_buf.size());
    for(uint i=0; i<x_buf.size(); i++)
    {
      x_buf_opt[i].p = es0.R * x_buf[i].p + es0.p;
      x_buf_opt[i].R = es0.R * x_buf[i].R;
      x_buf_opt[i].t = x_buf[i].t;
    }

    // get lidar and camera data
    // std::vector<IMUST> lidar_pose_opt, camera_pose_opt;
    // std::vector<pcl::PointCloud<PointType>::Ptr> lidar_points_opt, camera_points_opt;
    // std::cout << "Splitting multi-modal data..." << std::endl;
    // split_multi_modal_data(x_buf_opt, pl_fulls, lidar_pose_opt, camera_pose_opt, lidar_points_opt, camera_points_opt);

    printf("\nRefined point cloud is publishing...\n");
    malloc_trim(0);
    data_show(x_buf_opt, pl_fulls);
    printf("\nRefined point cloud is published.\n");
    // 存储数据
    std::cout << "chunk " << i << " cross-modal BA completed" << std::endl;
    std::cout << "saving results to " << result_path << std::endl;
    std::vector<IMUST> camera_pose_opt;
    // camera_pose_opt = x_buf_opt;
    for(int j = 0; j < x_buf_opt.size(); j++) {
    // for(int j = lidar_pose_num; j < x_buf_opt.size(); j++) {

        camera_pose_opt.push_back(x_buf_opt[j]);
    }
    std::cout << "save camera pose size: " << camera_pose_opt.size() << std::endl;
    save_TUM_Pose(result_path + "chunk" + std::to_string(i) + ".txt", camera_pose_opt);
    //Jack note，todo: 存储，同时准备HBA的数据
    camera_pose_opt.clear();
    pl_fulls.clear();
    x_buf.clear();
    //Jack 延时
    ros::Duration(15.0).sleep();
  }

  ros::spin();
  return 0;

}