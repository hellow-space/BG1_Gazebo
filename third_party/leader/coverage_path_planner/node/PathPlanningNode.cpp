//
// Created by colin on 2025/12/15.
//

#include "PathPlanningNode.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <errno.h>

namespace ikitbot::planning {
PathPlanning::PathPlanning(ros::NodeHandle& nh) : nh_(nh),
   grid_ready_(false),
   grid_updated_ready_(false) {
 
    // init params
   ros::NodeHandle private_nh("~");
   private_nh.param<int>("obstacle_inflate", obstacle_inflate_, 6);
   private_nh.param<double>("restrict_expan_dis", restrict_expan_dis_, 0.0);
   private_nh.param<double>("boundary_shrink_dis", boundary_shrink_dis_, 0.0);
   private_nh.param<double>("task_area_shrink_dis", task_area_shrink_dis_, 0.0);
   private_nh.param<bool>("debug", debug_, false);
   private_nh.param<bool>("publish_paths", publish_paths_, false);
   private_nh.param<double>("path_width", path_width_, 0.05);
   private_nh.param<bool>("boundary_path", boundary_path_, false);
   private_nh.param<bool>("island_path", island_path_, false);
   private_nh.param<double>("island_min_area", island_min_area_, 2.0);
   private_nh.param<double>("island_path_shrink_dis", island_path_shrink_dis_, 0.10);
   private_nh.param<int>("min_path_num", min_path_num_, 3);
   
   
   smooth_boundary_path_=true;
   ROS_INFO("debug=%d,publish_paths=%d",
    static_cast<int>(debug_),
    static_cast<int>(publish_paths_));
   ROS_INFO("obstacle_inflate:%d,restrict_expan_dis=%0.4f,boundary_shrink_dis=%0.4f,task_area_shrink_dis=%0.4f",
           obstacle_inflate_, restrict_expan_dis_,boundary_shrink_dis_,task_area_shrink_dis_);
  ROS_INFO("island_path=%d,island_min_area=%0.4f,island_path_shrink_dis=%0.4f",
    static_cast<int>(island_path_),island_min_area_,island_path_shrink_dis_);

  costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 1, &PathPlanning::grid_callback, this);

  costmap_updated_sub_ = nh_.subscribe<map_msgs::OccupancyGridUpdate>(
      "move_base_flex/global_costmap/costmap_updates", 1,
      &PathPlanning::grid_updated_callback, this);

  coverage_path_plan_srv_ = nh_.advertiseService(
      "/plan_backshaped_path_srv", &PathPlanning::coveragePathPlanSrv, this);
  
  boundary_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "/coverage_boundary_markers", 10, true);

  coverage_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "/coverage_path_markers", 10, true);

  island_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "/island_path_markers", 10, true);


  if( debug_ ) {
    const char* path = "/home/pi/tmp";
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        if (mkdir(path, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) == 0) {
            ROS_INFO("Success:tmp directory created.");
        } else {
           ROS_INFO("Fail:tmp directory created !");
        }
    }
  } 
  
}

void PathPlanning::grid_callback(nav_msgs::OccupancyGridConstPtr const &msg) {
  grid_ready_ = false;
  grid_ = *msg;
  grid_ready_ = true;
  ROS_INFO("recvied the "
           "map.width=%d,height=%d,resolution=%0.4f,origint(%0.4f,%0.4f)",
           msg->info.width, msg->info.height, msg->info.resolution,
           msg->info.origin.position.x, msg->info.origin.position.y);
}

void PathPlanning::grid_updated_callback(
    map_msgs::OccupancyGridUpdateConstPtr const &msg) {
  // if (!grid_ready_) return;

  // grid_updated_ready_ = false;
  // int index = 0;
  // for (unsigned y = msg->y; y < msg->y + msg->height; ++y) {
  //   for (unsigned x = msg->x; x < msg->x + msg->width; ++x) {
  //     grid_.data[y * grid_.info.width + x] = msg->data[index++];
  //   }
  // }

  // grid_updated_ready_ = true;
}

bool PathPlanning::coveragePathPlanSrv(
    robot_msg::CoveragePathPlan::Request &req,
    robot_msg::CoveragePathPlan::Response &resp) {
  ros::Time start_time = ros::Time::now();
  ROS_INFO(" ENTRY plan_backshaped_path_srv, regions sz: %d",
           static_cast<int>(req.clean_regions.size()));

  if (req.clean_regions.size() < 1) {
    resp.result = false;
    resp.failure_reason = "no_region_info_fail";
    ROS_ERROR("no_region_info_fail ... ");
    return true;
  }

  if (req.map_location == "") {
    resp.result = false;
    resp.failure_reason = "map id ";
    ROS_ERROR("on map id input ... ");
    return true;
  }

  if (!grid_ready_) {
    resp.result = false;
    resp.failure_reason = "map_subscribe_fail";
    ROS_ERROR("map_subscribe_fail!");
    return true;
  }

  // 创建协助类
  auto task_helper_ptr= std::make_shared<TaskHelper>(); 
  cv::Mat cv_costmat;
  std::shared_ptr<costmap_2d::Costmap2D> costmap_ptr;
  //将grid转成costmat,加载禁区等区域信息，填充加载禁区等区域到costmat中
  costmap_ptr = task_helper_ptr->initPlanTask(req.map_location, grid_, cv_costmat,restrict_expan_dis_,debug_,resp.failure_reason);
                                       
  if (costmap_ptr == nullptr) {
    resp.result = false;
    resp.failure_reason = "int task map fail.";
    ROS_ERROR("int task map fail");
    return true;
  }

    // 获得充电桩
  // cv::Point2f docker_pose;
  // bool is_docker_exist = false;
  // is_docker_exist = task_helper_ptr->getDocker(docker_pose);
 
  // 起点的设置
  cv::Point2f start_pose(0,0);
  if(req.clean_regions[0].start_position.size()>1){
    start_pose.x=req.clean_regions[0].start_position.at(0);
    start_pose.y=req.clean_regions[0].start_position.at(1);
    ROS_INFO("input start_position[%0.4f,%0.4f]",start_pose.x,start_pose.y);
  }


    /**** 这里创建 沿边marks/回字marks */
  visualization_msgs::MarkerArray boundary_markers;
  visualization_msgs::MarkerArray coverage_markers;
  visualization_msgs::MarkerArray island_markers;

  ros::Time marker_stamp = ros::Time::now();
  resp.clean_regions.clear();
  resp.fail_region_id.clear();

  //  遍历每个清洁区进行规划
  for (int reg = 0; reg < req.clean_regions.size(); ++reg) {
    ROS_INFO("plan_backshaped_path_srv , [%d] of [%d] , region_id [%d],path type = % d ", 
                  reg, 
                  static_cast<int>(req.clean_regions.size()),
                  req.clean_regions[reg].region_id,
                  static_cast<int>(req.clean_regions[reg].path_type));

  
    // 解析传入的清洁区的信息
    CleanRegionInfo clean_info;
    if (!loadCleanRegionFromSvr(task_helper_ptr,req.clean_regions[reg], clean_info)) {
      ROS_WARN("region id[%d] msg is wrong ", req.clean_regions[reg].region_id);
      resp.fail_region_id.push_back(req.clean_regions[reg].region_id);
      continue;
    }
    // 按不同的模式，将相交的地毯标注在地图上
    task_helper_ptr->markCarpetRegionOnMat(clean_info, cv_costmat,true);
 

    // 获得沿边路径及工作区的costmat
    cv::Mat task_costmat;
    if(false == task_helper_ptr->getTasMatAndBoudPath(cv_costmat, task_area_shrink_dis_, boundary_shrink_dis_, obstacle_inflate_,
        clean_info, task_costmat, smooth_boundary_path_)){
      ROS_WARN("region id[%d] boundary is wrong!", req.clean_regions[reg].region_id);
      resp.fail_region_id.push_back(req.clean_regions[reg].region_id);
      continue;
    }

  
    // 获得zigzag/回字形路径 /island 路径
    CoveragePathPlanner path_planer;
    std::vector<nav_msgs::Path> paths;
    paths = path_planer.get_path_interface(task_costmat,start_pose,costmap_ptr,clean_info,
      island_path_,island_min_area_,island_path_shrink_dis_,min_path_num_,obstacle_inflate_,debug_);
  
    robot_msg::RegionClean clean_msg;
    clean_msg.region_id = clean_info.region_id;
    clean_msg.line_speed=clean_info.line_speed;

    clean_msg.boundary_path.clear();
    clean_msg.coverage_path.clear();
    clean_msg.island_path.clear();
    std::ostringstream oss_boundary;
    std::ostringstream oss_coverage;
    std::ostringstream oss_island;
    oss_boundary << std::fixed << std::setprecision(2);
    oss_coverage << std::fixed << std::setprecision(2);
    oss_island << std::fixed << std::setprecision(2);

    size_t sz=paths.size();
    if(sz<=0 ) {
      ROS_WARN("region id[%d] get paths fail! ", req.clean_regions[reg].region_id);
      continue;
    }

    // ----------- 处理 coverage path  --------
    auto cover_path=paths[0];
    for (size_t i = 0; i < cover_path.poses.size(); ++i) {
      const auto &p = cover_path.poses[i].pose.position;
      oss_coverage << p.x << "," << p.y;
      if (i < cover_path.poses.size() - 1)
        oss_coverage << ",";
    }
    clean_msg.coverage_path = oss_coverage.str();
    
    if(publish_paths_){
       buildCoverageMarker(cover_path,coverage_markers,reg,marker_stamp);
    }

    // ----------- 处理 island path  --------
    if( island_path_ ){
      if(sz>1){
        auto island_path=paths[1];
        for (size_t i = 0; i < island_path.poses.size(); ++i) {
          const auto &p = island_path.poses[i].pose.position;
          oss_island << p.x << "," << p.y;
          if (i < island_path.poses.size() - 1)
            oss_island << ",";
        }

        clean_msg.island_path = oss_island.str();

        if(publish_paths_){
          buildIslandMarker(island_path,island_markers,reg,marker_stamp);
        }
      }
      
    }

    //----------- 处理 boundary path  ------------
    if( boundary_path_ ){
      int boundary_sz =static_cast<int>(clean_info.boundary_path.size());
      for (int z = 0; z < boundary_sz; ++z) {
      float wx, wy;
      task_helper_ptr->mapToWorld(clean_info.boundary_path.at(z).x,
                clean_info.boundary_path.at(z).y, wx, wy);
      oss_boundary << wx << "," << wy;
      if (z < boundary_sz - 1) oss_boundary << ",";
      }
      clean_msg.boundary_path = oss_boundary.str();

      if(publish_paths_){
        buildBoundaryMarker(task_helper_ptr,clean_info,boundary_markers,reg,marker_stamp);
      }
    }
   
    // 将最后的点设置为下个清洁区参数起点
    if(cover_path.poses.size()){
      start_pose.x=cover_path.poses.back().pose.position.x;
      start_pose.y=cover_path.poses.back().pose.position.y;
    }

    // 将clean_msg push 结果中
    clean_msg.clean_area =task_helper_ptr->getRegionCleanArea(task_costmat,clean_info.boundary_path);
    clean_msg.time_require=task_helper_ptr->getRegionCleanTimeRequire(clean_msg);
    ROS_INFO("reg[%d],area=%0.4f.require time=%0.4f",clean_msg.region_id,clean_msg.clean_area,clean_msg.time_require);
    resp.clean_regions.push_back(clean_msg);
  }

  // 执行结果生成
  task_helper_ptr->assessmentCleanTask(req,resp);

  // 发布结果 
  if(publish_paths_){
    boundary_marker_pub_.publish(boundary_markers);
    coverage_marker_pub_.publish(coverage_markers);
    island_marker_pub_.publish(island_markers);
  }

  ros::Time end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  ROS_WARN("Backshaped Plan took %f seconds", duration.toSec());


  resp.result = true;
  return true;
}
bool PathPlanning::loadCleanRegionFromSvr(const std::shared_ptr<TaskHelper>& helper_ptr,
                                          const robot_msg::RegionClean &req,
                                          CleanRegionInfo &reg_info) {
  reg_info.parent_region_id = -1;
  reg_info.region_id = req.region_id;
  reg_info.path_type = req.path_type;
  reg_info.mode = req.mode;
  reg_info.line_speed = req.line_speed;
  if (reg_info.line_speed < 0.01f)
    reg_info.line_speed = 0.5f;
  reg_info.zigzag_space = req.zigzag_space;
  if (reg_info.zigzag_space == 0)
    reg_info.zigzag_space = 8;

  reg_info.number_erosions = obstacle_inflate_;

  reg_info.first_path_space = 0;
  reg_info.path_to_obs_dis = 0;
  reg_info.min_line_length = 0;

  ROS_INFO("region_id = %d", req.region_id);
  ROS_INFO("mode = %d", req.mode);
  ROS_INFO("line_speed = %f", req.line_speed);
  ROS_INFO("zigzag_space = %d", req.zigzag_space);
  ROS_INFO("number_erosions = %d", reg_info.number_erosions);
  // ROS_INFO("first_path_space = %d", reg_info.first_path_space);
  // ROS_INFO("path_to_obs_dis = %d", reg_info.path_to_obs_dis);
  // ROS_INFO("min_line_length = %d", reg_info.min_line_length);

  // generate task cell
  if (req.clean_region.size() < 6) {
    ROS_ERROR(" task_cell_vertices , id: %d", req.region_id);
    return false;
  } else {
    for (int i = 0; i < req.clean_region.size() - 1; ++i) {
      int mx, my;
      helper_ptr->worldToMap(req.clean_region[i], req.clean_region[i + 1], mx,
                              my);
      reg_info.task_cell_vertices.push_back(cv::Point(mx, my));
      // ROS_INFO("clean_region id: %d :: world:[%.3f %.3f] map:[%d %d]",
      //          req.region_id, req.clean_region[i], req.clean_region[i + 1],
      //          mx, my);
      i++;
    }
  }

  return true;
}
void PathPlanning::buildBoundaryMarker(const std::shared_ptr<TaskHelper>& helper_ptr,
                                        const CleanRegionInfo &clean_info,
                                        visualization_msgs::MarkerArray &out_array,
                                        const int marker_id,
                                        const ros::Time &stamp) {

  if (clean_info.boundary_path.empty()) {
    return;
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = stamp;
  mk.ns = "boundary_paths";
  mk.id = marker_id+1;
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::ADD;

  mk.scale.x =path_width_;

  // 红色：沿边
  mk.color.r = 1.0;
  mk.color.g = 0.2;
  mk.color.b = 0.2;
  mk.color.a = 1.0;

  for (const auto &pt : clean_info.boundary_path) {
    float wx, wy;
    helper_ptr->mapToWorld(pt.x, pt.y, wx, wy);

    geometry_msgs::Point p;
    p.x = wx;
    p.y = wy;
    p.z = 0.05;
    mk.points.push_back(p);
  }

  out_array.markers.push_back(mk);
}
void PathPlanning::buildIslandMarker(const nav_msgs::Path &path,
                                        visualization_msgs::MarkerArray &out_array,
                                        const int marker_id,
                                        const ros::Time &stamp) {

  if (path.poses.empty()) {
    return;
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = stamp;
  mk.ns = "island_paths";
  mk.id = marker_id+1;
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::ADD;

  mk.scale.x = path_width_;

  // 蓝绿色：coverage
  mk.color.r = 0.1;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 1.0;

  for (const auto &ps : path.poses) {
    geometry_msgs::Point p = ps.pose.position;
    p.z = 0.03;
    mk.points.push_back(p);
  }

  out_array.markers.push_back(mk);
}
void PathPlanning::buildCoverageMarker(const nav_msgs::Path &path,
                                      visualization_msgs::MarkerArray &out_array,
                                      const int marker_id,
                                      const ros::Time &stamp) {

  if (path.poses.empty()) {
    return;
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = stamp;
  mk.ns = "coverage_paths";
  mk.id = marker_id+1;
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::ADD;

  mk.scale.x = path_width_;

  // 蓝绿色：coverage
  mk.color.r = 0.1;
  mk.color.g = 0.8;
  mk.color.b = 1.0;
  mk.color.a = 1.0;

  for (const auto &ps : path.poses) {
    geometry_msgs::Point p = ps.pose.position;
    p.z = 0.03;
    mk.points.push_back(p);
  }

  out_array.markers.push_back(mk);
}

} // namespace ikitbot::planning

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle nh;
  ROS_INFO("coverage path plan node start.");
  ikitbot::planning::PathPlanning path_planning(nh);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
