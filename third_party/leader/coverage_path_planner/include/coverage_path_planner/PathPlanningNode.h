//
// Created by colin on 2025/12/12.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <std_srvs/Empty.h>

#include "CoveragePathPlanner.h"
#include "task_helper.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <robot_msg/CoveragePathPlan.h>
namespace ikitbot::planning {
//#define DEBUG_MODE

class PathPlanning {
public:
  PathPlanning(ros::NodeHandle& nh);

  ~PathPlanning() = default;

private:
  void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
  void grid_updated_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
  bool coveragePathPlanSrv(robot_msg::CoveragePathPlan::Request &req,
                           robot_msg::CoveragePathPlan::Response &resp);
  bool loadCleanRegionFromSvr(const std::shared_ptr<TaskHelper>& helper_ptr,
                              const robot_msg::RegionClean &req,
                              CleanRegionInfo &reg_info);
  void buildCoverageMarker(const nav_msgs::Path &path,
                          visualization_msgs::MarkerArray &out_array,
                          const int marker_id,
                          const ros::Time &stamp);
  void buildBoundaryMarker(const std::shared_ptr<TaskHelper>& helper_ptr,
                          const CleanRegionInfo &clean_info,
                          visualization_msgs::MarkerArray &out_array,
                          const int marker_id,
                          const ros::Time &stamp);
void buildIslandMarker(const nav_msgs::Path &path,
                          visualization_msgs::MarkerArray &out_array,
                          const int marker_id,
                          const ros::Time &stamp);
private:
  ros::NodeHandle& nh_;
  ros::ServiceServer coverage_path_plan_srv_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_updated_sub_;
  ros::Publisher  boundary_marker_pub_;
  ros::Publisher coverage_marker_pub_;
  ros::Publisher island_marker_pub_;

  nav_msgs::OccupancyGrid grid_;
  std::atomic_bool grid_ready_;
  std::atomic_bool grid_updated_ready_;
 
private:
  double restrict_expan_dis_;  // 禁区扩张距离
  double task_area_shrink_dis_;  // 工作区收缩距离
  double boundary_shrink_dis_; // 沿边缩进距离,取轮廓后使用多边形进行缩进
  int obstacle_inflate_; // (grid) 障碍物膨胀的grid
  bool smooth_boundary_path_=false;
  bool debug_=false;
  bool publish_paths_=false;
  double path_width_=0.05;
  bool boundary_path_=false;
  bool island_path_=false;
  double island_min_area_=2.0;
  double island_path_shrink_dis_=0.10;
  int min_path_num_=3;

};
} // namespace ikitbot::planning
