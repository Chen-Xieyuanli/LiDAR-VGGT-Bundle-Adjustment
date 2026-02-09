cmake_minimum_required(VERSION 3.0.2)
project(session_align)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -g -ggdb")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  # cv_bridge
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
# find_package(OpenCV REQUIRED)
find_package(Threads)
# find_package(Ceres REQUIRED)
# find_package(GSL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offline_refinement
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
)



# # loop_graph_opt.cpp
# add_executable(loop_graph_opt src/loop_graph_opt.cpp)
# target_link_libraries(loop_graph_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

# # factor_graph_opt.cpp
# add_executable(factor_graph_opt src/factor_graph_opt.cpp)
# target_link_libraries(factor_graph_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

# session graph optimization
add_executable(session_graph_opt src/session_graph_opt.cpp)
target_link_libraries(session_graph_opt ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)


add_executable(pcd_publisher src/view_pcd.cpp)
target_link_libraries(pcd_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

