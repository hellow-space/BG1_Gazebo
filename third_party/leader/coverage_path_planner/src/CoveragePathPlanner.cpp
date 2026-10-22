//
// Created by tony on 2022/10/25.
//

#include "CoveragePathPlanner.h"

namespace ikitbot::planning {
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

void CoveragePathPlanner::mapToWorld(const unsigned int mx,
                                     const unsigned int my, double &wx,
                                     double &wy) {
  if (rotated_costmap_) {
    auto size_y = rotated_costmap_->getSizeInCellsY();
    rotated_costmap_->mapToWorld(mx, size_y - my - 1, wx, wy);
  }
}
void CoveragePathPlanner::worldToMap(const double wx, const double wy,
                                     unsigned int &mx, unsigned int &my) {
  if (rotated_costmap_) {
    auto size_y = rotated_costmap_->getSizeInCellsY();
    rotated_costmap_->worldToMap(wx, wy, mx, my);
    my = size_y - my - 1;
  }
}

void CoveragePathPlanner::ini_task(
    const cv::Mat &costmat,
    std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
    const CleanRegionInfo &cr_info) {

  // copy costmap and costmat
  rotated_costmap_ = costmap;
  work_binmat_ = costmat2Binmat(costmat);

  // init params
  model_size_cell_ = cr_info.zigzag_space;
  cell_free_thres_ = 10;
  resolution_ = costmap->getResolution();
  lethal_cost_ = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  if (cr_info.path_type == 1)
    clean_area_type_ = CleanAreaType::backshaped;
  else
    clean_area_type_ = CleanAreaType::zigzag;

  ROS_INFO("model_size_cell_=%d,cell_free_thres_=%d,path_type=%d",
           model_size_cell_, cell_free_thres_,
           static_cast<int>(clean_area_type_));
  // debug show input costmat
  show_cost_plygon(costmat, cr_info.boundary_path, "s6_input_costmap");

  // 沿边路径转ros path,并为每个点计算方向
  nav_msgs::Path tmp;
  tarnCVPath2RosPath(cr_info.boundary_path, tmp);
  tmp = BezierTrajectoryGeneratorWaypoint::calculate_path_orientation(tmp, true)
            .value();
  // 将闭合后多的点删除
  origin_border_ = polygon_cutter(tmp);
  planning_polygon_ = origin_border_;

  //旋转
  // angle_ = calculate_polygon_inclination(planning_polygon_, FIL_TOLER);
  angle_ = 0.0;
  rotate_costmat(costmat, angle_);

  //
  initialized_ = initialize_mats();
}

std::vector<nav_msgs::Path> 
CoveragePathPlanner::get_path_interface(
    const cv::Mat &costmat,const cv::Point2f& start,
    std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
    const CleanRegionInfo &cr_info,bool has_island_path,
    double island_min_area,double island_path_shrink_dis,
    const int min_path, const int obs_inflate,bool debug) {
  debug_ = debug;
  obs_inflate_=obs_inflate;
  ROS_INFO("get_path_interface start:");

  std::vector<nav_msgs::Path> result_paths;

  if (costmap == nullptr) {
    ROS_ERROR("cost map is null!");
    return result_paths;
  }

  // int task
  ini_task(costmat, costmap, cr_info);

  // prepare map
  // int max_inflate=std::round(0.30 / 0.05);
  int max_inflate=4;
  int need_inflat=max_inflate-obs_inflate_;
  astar_fast_planner_.prepareMap(work_binmat_, 1.0, need_inflat, 0.05);
  

  // zigzag/回字形 路径
  nav_msgs::Path cover_path;
  std::vector<nav_msgs::Path> raw_cover_paths = get_coverage_paths(start,min_path);
  show_path(rotated_costmat_, raw_cover_paths, "s10_raw_cover_paths.png");
  cover_path=get_merged_path(raw_cover_paths);
  result_paths.push_back(cover_path);
  // island 路径
  if( has_island_path){
    nav_msgs::Path island_path;
    std::vector<nav_msgs::Path> raw_island_paths;
    cv::Point stat_pt(0,0);
    if(cover_path.poses.size()){
      unsigned mx,my;
      double wx = cover_path.poses.back().pose.position.x;
      double wy = cover_path.poses.back().pose.position.y;
      auto size_y = rotated_costmap_->getSizeInCellsY();
      if (costmap->worldToMap(wx, wy, mx, my)) {
        stat_pt.x = static_cast<int>(mx);
        stat_pt.y = static_cast<int>(size_y - my - 1);
      }
    }
    raw_island_paths=get_island_paths_new(work_binmat_,costmap,cr_info.boundary_path,
      island_min_area,island_path_shrink_dis,stat_pt);
    show_path(rotated_costmat_, raw_island_paths, "s11_raw_island_paths.png");
    island_path=get_merged_path(raw_island_paths,true);
    result_paths.push_back(island_path);
  }
  
  return result_paths;
}

nav_msgs::Path CoveragePathPlanner::get_merged_smooth_path(const std::vector<nav_msgs::Path> & raw_paths, bool is_island) {
    nav_msgs::Path result_path;
    result_path.header.frame_id = "map";
    
    const size_t n = raw_paths.size();
    ROS_INFO("get paths size=%d", static_cast<int>(n));

    if (n == 0) return result_path;
    if (n == 1) {
        result_path = raw_paths[0];
        show_result_path(rotated_costmat_, result_path, "result_paths.png", is_island);
        return result_path;
    }

    // 将 merged 改为像素点集
    std::vector<cv::Point> merged_pixel_path;
    std::vector<cv::Point> astar_pts;

    // 辅助 lambda：将世界坐标 Path 转换为像素点集
    auto append_nav_path_to_pixels = [&](const nav_msgs::Path& path, std::vector<cv::Point>& out_pixels) {
        for (const auto& pose_stamped : path.poses) {
            unsigned mx, my;
            if (rotated_costmap_->worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, mx, my)) {
                int px = static_cast<int>(mx);
                int py = static_cast<int>(rotated_costmat_.rows - static_cast<int>(my) - 1);
                // 简单去重：如果当前点和上一个点相同，则不插入
                if (out_pixels.empty() || out_pixels.back().x != px || out_pixels.back().y != py) {
                    out_pixels.push_back(cv::Point(px, py));
                }
            }
        }
    };

    // 辅助 lambda：获取 Pose 对应的像素点
    auto get_pixel_from_pose = [&](const geometry_msgs::Pose& pose) -> cv::Point {
        unsigned mx, my;
        rotated_costmap_->worldToMap(pose.position.x, pose.position.y, mx, my);
        int px = static_cast<int>(mx);
        int py = static_cast<int>(rotated_costmat_.rows - static_cast<int>(my) - 1);
        return cv::Point(px, py);
    };

    // 1) 寻找第一个可连接的对
    int start_index = -1;
    for (size_t i = 0; i < n - 1; ++i) {
        cv::Point p_start = get_pixel_from_pose(raw_paths[i].poses.back().pose);
        cv::Point p_goal = get_pixel_from_pose(raw_paths[i+1].poses.front().pose);

        astar_pts.clear();
        astar_fast_planner_.planPathFast(p_start, p_goal, 4, astar_pts);

        if (!astar_pts.empty()) {
            append_nav_path_to_pixels(raw_paths[i], merged_pixel_path);
            merged_pixel_path.insert(merged_pixel_path.end(), astar_pts.begin(), astar_pts.end());
            append_nav_path_to_pixels(raw_paths[i+1], merged_pixel_path);
            start_index = static_cast<int>(i + 1);
            ROS_INFO("Initialized merged pixels with pair (%lu,%lu)", i, i + 1);
            break;
        }
    }

    if (start_index == -1) {
        ROS_WARN("No connectable adjacent path pairs found.");
        return result_path;
    }

    // 2) 逐个连接后续路径
    for (int j = start_index + 1; j < static_cast<int>(n); ++j) {
        cv::Point p_start = merged_pixel_path.back();
        cv::Point p_goal = get_pixel_from_pose(raw_paths[j].poses.front().pose);

        astar_pts.clear();
        astar_fast_planner_.planPathFast(p_start, p_goal, 4, astar_pts);

        if (astar_pts.empty()) {
            show_miss_path(work_binmat_, p_start, p_goal, j, raw_paths[j], is_island);
            continue;
        }

        merged_pixel_path.insert(merged_pixel_path.end(), astar_pts.begin(), astar_pts.end());
        append_nav_path_to_pixels(raw_paths[j], merged_pixel_path);
    }

    // 3) 关键步骤：调用 smooth 优化尖锐转角
    std::vector<cv::Point> smoothed_pixels;
    astar_fast_planner_.smoothPath3(merged_pixel_path, smoothed_pixels);

  
    // 4) 将平滑后的像素点集转回 result_path (World 坐标)
    auto size_y = rotated_costmap_->getSizeInCellsY();
    for (const auto &pix : smoothed_pixels) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        rotated_costmap_->mapToWorld(pix.x, size_y - pix.y - 1, ps.pose.position.x, ps.pose.position.y);
        ps.pose.position.z = 0.0; // 还原高度
        result_path.poses.push_back(ps);
    }

    ROS_INFO("Final merged and smoothed path size: %d", static_cast<int>(result_path.poses.size()));
    show_result_path(rotated_costmat_, result_path, "result_paths.png", is_island);

    return result_path;
}

nav_msgs::Path CoveragePathPlanner::get_merged_path(const std::vector<nav_msgs::Path> & raw_paths,bool is_island){
  nav_msgs::Path result_path;
  nav_msgs::Path merged;
  const size_t n = raw_paths.size();
  ROS_INFO("get_merged_path size=%d", static_cast<int>(n));

  if (n == 0)
    return result_path;
  else if (n == 1) {
    result_path = raw_paths[0];
    show_result_path(rotated_costmat_, result_path, "result_paths.png",is_island);
    return result_path;
  }
  merged.header.frame_id = "map";
  std::vector<cv::Point> astar_pts;
  std::vector<cv::Point> merged_debug_poits;

  auto make_pixel = [&](const geometry_msgs::Pose &pose,
                        cv::Point &out_pt) -> bool {
    unsigned mx, my;
    if (!rotated_costmap_->worldToMap(pose.position.x, pose.position.y, mx,
                                      my)) {
      // worldToMap 可能返回 false，但旧代码未检查，这里我们也允许失败但 log
      ROS_WARN("worldToMap failed for (%.3f, %.3f)", pose.position.x,
               pose.position.y);
      // still convert to integer indices clamped
    }
    int px = static_cast<int>(mx);
    int py = static_cast<int>(rotated_costmat_.rows - static_cast<int>(my) - 1);
    // clamp
    px = std::max(0, std::min(px, rotated_costmat_.cols - 1));
    py = std::max(0, std::min(py, rotated_costmat_.rows - 1));
    out_pt = cv::Point(px, py);
    return true;
  };

  // 1) 如果 merged 为空，从 raw_paths 找到首个可连接的相邻对
  int start_index =      -1; 
  for (size_t i = 0; i < n - 1; ++i) {
    // prepare start and goal pixels
    const geometry_msgs::Pose &end_curr = raw_paths[i].poses.back().pose;
    const geometry_msgs::Pose &beg_next = raw_paths[i + 1].poses.front().pose;

    cv::Point p_start, p_goal;
    make_pixel(end_curr, p_start);
    make_pixel(beg_next, p_goal);

    astar_pts.clear();
    // astar_planner_.planPath(work_binmat_, p_start, p_goal, 0.5, 0.0, 0.05, 4,
    // &astar_pts);
    astar_fast_planner_.planPathFast(p_start, p_goal, 4, astar_pts);

    ROS_INFO("trying pair (%lu,%lu) astar size=%d", i, i + 1,
             static_cast<int>(astar_pts.size()));
    // show_astart_path(work_binmat_, astar_pts, static_cast<int>(i));

    if (!astar_pts.empty()) {
      // found a connectable pair: initialize merged = path[i] + astar +
      // path[i+1]
      merged.poses.insert(merged.poses.end(), raw_paths[i].poses.begin(),
                          raw_paths[i].poses.end());
      // insert astar points
      auto size_y = rotated_costmap_->getSizeInCellsY();
      for (const auto &pix : astar_pts) {
        geometry_msgs::PoseStamped ps;
        rotated_costmap_->mapToWorld(pix.x, size_y - pix.y - 1,
                                     ps.pose.position.x, ps.pose.position.y);
        ps.header.frame_id = "map";
        if( debug_ )
          ps.pose.position.z = 1.0;
        merged.poses.push_back(ps);
      }
      merged.poses.insert(merged.poses.end(), raw_paths[i + 1].poses.begin(),
                          raw_paths[i + 1].poses.end());
      start_index = static_cast<int>(i + 1);
      ROS_INFO("Initialized merged with pair (%lu,%lu)", i, i + 1);
      break;
    } // else try next pair
  }

  // 如果没有找到任何可连接的对 -> 无可用 merged，直接返回空
  if (start_index == -1) {
    ROS_WARN("No connectable adjacent path pairs found. Return idx=0 path.");
    result_path = raw_paths[0];
    return result_path;
  }

  // 2) merged 已初始化，从 start_index 的下一个路径开始尝试逐个连接
  for (int j = start_index + 1; j < static_cast<int>(n); ++j) {
    // 起点为 merged 的最后一个点
    if (merged.poses.empty()) {
      ROS_WARN("Merged became empty unexpectedly.");
      break;
    }
    const geometry_msgs::Pose &merged_last = merged.poses.back().pose;
    const geometry_msgs::Pose &next_first = raw_paths[j].poses.front().pose;

    cv::Point p_start, p_goal;
    make_pixel(merged_last, p_start);
    make_pixel(next_first, p_goal);

    astar_pts.clear();
    // astar_planner_.planPath(work_binmat_, p_start, p_goal, 0.5, 0.0, 0.05, 4,
    // &astar_pts);
    astar_fast_planner_.planPathFast(p_start, p_goal, 4, astar_pts);
  //  ROS_INFO("connect merged->path[%d], astar size=%d", j,
  //            static_cast<int>(astar_pts.size())); 
    // show_astart_path(work_binmat_, astar_pts, j);

    if (astar_pts.empty()) {
      // 连接失败 -> 跳过该 raw_paths[j]
      show_miss_path(work_binmat_, p_start, p_goal, j, raw_paths[j],is_island);
      ROS_WARN("Cannot connect merged to path[%d], skipping path[%d].", j, j);
      continue;
    }

    // 连接成功 -> 插入 astar 段和 raw_paths[j]
    auto size_y = rotated_costmap_->getSizeInCellsY();
    for (const auto &pix : astar_pts) {
      geometry_msgs::PoseStamped ps;
      rotated_costmap_->mapToWorld(pix.x, size_y - pix.y - 1,
                                   ps.pose.position.x, ps.pose.position.y);
      ps.header.frame_id = "map";
      if( debug_ )
        ps.pose.position.z = 1.0;
      merged.poses.push_back(ps);
    }
    merged.poses.insert(merged.poses.end(), raw_paths[j].poses.begin(),
                        raw_paths[j].poses.end());
    // ROS_INFO("Appended path[%d] to merged.", j);
  }

  // 保存
  
  result_path = merged;

  ROS_INFO("island=%d,result_path size=%d",
         static_cast<int>(is_island), 
       static_cast<int>(result_path.poses.size()));
  show_result_path(rotated_costmat_, result_path, "result_paths.png",is_island);

  if( debug_ ){
    for(int i=0; i < result_path.poses.size()-1; i++ ){
      result_path.poses.at(i).pose.position.z = 0;
    }
  }
  
  return result_path;
}
nav_msgs::Path
CoveragePathPlanner::polygon_cutter(nav_msgs::Path const &polygon) {
  nav_msgs::Path cut_polygon = polygon;
  /**Revise the Goal**/
  auto check_dist2 = CHECK_DIST * CHECK_DIST;
  if (dist2(cut_polygon.poses.back(), cut_polygon.poses.front()) <
          check_dist2 &&
      cut_polygon.poses.size() > CHECK_MAX) {
    auto dist2_min = FLT_MAX;
    auto iter = cut_polygon.poses.end();
    for (auto it = cut_polygon.poses.end() - CHECK_MAX;
         it != cut_polygon.poses.end(); ++it) {
      auto dist2_tmp = dist2((*it), cut_polygon.poses.front());
      if (dist2_tmp > check_dist2)
        continue;
      if (dist2_min > dist2_tmp) {
        dist2_min = static_cast<float>(dist2_tmp);
        iter = it;
      }
    }

    cut_polygon.poses.erase(iter, cut_polygon.poses.end());
  }

  return cut_polygon;
}
void CoveragePathPlanner::show_cost_plygon(
    const cv::Mat costmat, const std::vector<cv::Point> &vertices,
    const std::string &tile) {
  if (false == debug_)
    return;
  cv::Mat tmp = costmat2Colormat(costmat);
  cv::drawContours(tmp, std::vector<std::vector<cv::Point>>(1, vertices), -1,
                   cv::Scalar(0, 0, 255), 2);

  std::string save_path = "/home/pi/tmp/" + tile + ".jpg";
  cv::imwrite(save_path, tmp);
}
cv::Mat CoveragePathPlanner::costmat2Colormat(const cv::Mat &costmat) {
  cv::Mat vis(costmat.rows, costmat.cols, CV_8UC3);
  for (int r = 0; r < costmat.rows; r++) {
    for (int c = 0; c < costmat.cols; c++) {
      uchar v = costmat.at<uchar>(r, c);
      cv::Vec3b color;
      if (v == costmap_2d::FREE_SPACE)
        color = {255, 255, 255}; // white
      else if (v == costmap_2d::NO_INFORMATION)
        color = {255, 255, 200}; // light cyan
      else if (v == costmap_2d::LETHAL_OBSTACLE)
        color = {0, 0, 0}; // black
      else if (v == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        color = {255, 0, 0}; // blue
      else if (v == costmap_2d::INSCRIBED_PLAN_OBSTACLE)
        color = {180, 180, 255}; // blue-gray
      else if (v == costmap_2d::INSCRIBED_POSSIBLY_OBSTACLE)
        color = {255, 150, 255}; // light purple
      else
        color = {128, 128, 128}; // gray

      vis.at<cv::Vec3b>(r, c) = color;
    }
  }
  return vis;
}
cv::Mat CoveragePathPlanner::costmat2Binmat(const cv::Mat &costmat) {
  cv::Mat bin(costmat.rows, costmat.cols, CV_8UC1);
  for (int r = 0; r < costmat.rows; r++) {
    for (int c = 0; c < costmat.cols; c++) {
      if (costmat.at<uchar>(r, c) == costmap_2d::FREE_SPACE)
        bin.at<uchar>(r, c) = 255;
      else
        bin.at<uchar>(r, c) = 0;
    }
  }
  return bin;
}
void CoveragePathPlanner::tarnCVPath2RosPath(
    const std::vector<cv::Point> &points, nav_msgs::Path &path) {
  for (const auto &point : points) {
    geometry_msgs::PoseStamped pt;
    double wx = 0;
    double wy = 0;
    mapToWorld(point.x, point.y, wx, wy);
    pt.pose.position.x = wx;
    pt.pose.position.y = wy;
    pt.pose.orientation.x = 0;
    pt.pose.orientation.y = 0;
    pt.pose.orientation.z = 0;
    pt.pose.orientation.w = 1;
    path.poses.push_back(pt);
  }
}
double CoveragePathPlanner::calculate_polygon_inclination(
    nav_msgs::Path const &polygon, double tolerance) {
  auto angle = 0.0;
  /**rotate**/
  if (polygon.poses.size() < 3) {
    angle = 0.0;
  } else {
    std::vector<cv::Point2f> cv_polygon;
    for (auto const &pose : polygon.poses) {
      cv_polygon.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    auto longest = 0.0;
    auto rect = cv::minAreaRect(cv_polygon);
    if (rect.size.height > rect.size.width) {
      longest = rect.size.height;
      angle = -(90 + rect.angle) / 180.0 * M_PI;
    } else {
      longest = rect.size.width;
      angle = -rect.angle / 180.0 * M_PI;
    }

    ROS_INFO("CoveragePathPlanner: Find longest edge = %.2f", longest);
    ROS_INFO("CoveragePathPlanner: Angle = %.2f deg, start idx = %d",
             angle * 180.0 / M_PI, 0);
  }

  return angle;
}
void CoveragePathPlanner::rotate_costmat(const cv::Mat &mat, double angle) {
  rotated_costmat_ = mat;
}
bool CoveragePathPlanner::initialize_mats() {
  cells_poses_ =
      get_free_cell_in_polygon(rotated_costmat_, scale_costmat_, free_space_);
  ROS_INFO("cells_poses_ size=%d", static_cast<int>(cells_poses_.size()));

  neural_costmat_ = cv::Mat(scale_costmat_.rows, scale_costmat_.cols, CV_32F);
  initialize_neural_mat(scale_costmat_, neural_costmat_);

  // debug
  show_free_cells(rotated_costmat_, free_space_, cells_poses_);
  return !neural_costmat_.empty();
}
void CoveragePathPlanner::initialize_neural_mat(cv::Mat const &src,
                                                cv::Mat &des) const {
  for (auto i = 0; i < des.rows; ++i) {
    for (auto j = 0; j < des.cols; ++j) {
      if (src.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE) {
        des.at<float>(i, j) = -FLT_MAX;
      } else {
        des.at<float>(i, j) =
            C_0 - static_cast<float>(j) / static_cast<float>(des.cols);
      }
    }
  }
}

std::vector<std::vector<SimpleCellIndex>>
CoveragePathPlanner::get_free_cell_in_polygon(
    cv::Mat const &src, cv::Mat &des, std::vector<CellIndex> &free_space) {
  auto free_space_calculator = std::make_unique<FreeSpaceCalculator>(
      cell_free_thres_, model_size_cell_, lethal_cost_);

  // free格子中允许有障碍物,格子内贴障碍物
  // free_space_calculator->get_free_cell_in_polygon(
  //     src, des, free_space, planning_polygon_, rotated_costmap_, true);

  // free格子中允许有障碍物,格子内尽量走中间
  // free_space_calculator->get_free_cell_in_polygon2(
  //     src, des, free_space, planning_polygon_, rotated_costmap_, true);

  free_space_calculator->get_free_cell_in_polygon_final(
      src, des, free_space, planning_polygon_, rotated_costmap_);

      
  return free_space_calculator->get_cells_poses();
}
void CoveragePathPlanner::show_free_cells(
    const cv::Mat &src, const std::vector<CellIndex> &free_space,
    const std::vector<std::vector<SimpleCellIndex>> &cells_poses) {
  if (false == debug_)
    return;
  // 1. 原图转 BGR，方便画彩色
  cv::Mat debug;
  debug = costmat2Colormat(src);

  // cv::Mat debug = cv::Mat(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
  // cv::cvtColor(src, debug, cv::COLOR_GRAY2BGR);

  ROS_INFO("free_space=%ld  poses=%ld", free_space.size(), cells_poses_.size());

  // 2. draw free cells
  for (size_t i = 0; i < free_space.size(); i++) {
    const auto &cell = free_space[i];
    int r0 = cell.row * model_size_cell_;
    int c0 = cell.col * model_size_cell_;

    cv::rectangle(debug, cv::Rect(c0, r0, model_size_cell_, model_size_cell_),
                  cv::Scalar(0, 0, 255), // red
                  1);

    // 3. draw poses inside the cell
    if (i < cells_poses.size()) {
      const auto &poses = cells_poses[i];
      for (auto const &p : poses) {
        // p.row, p.col 是图像坐标
        cv::circle(debug, cv::Point(p.col, p.row), 2,
                   cv::Scalar(255, 0, 0), // blue
                   -1);
      }
    }
  }
  // 4. 保存文件
  std::string file = "/home/pi/tmp/s8_freecell_result.jpg";
  if (!cv::imwrite(file, debug)) {
    std::cerr << "Failed to save debug image to: " << file << std::endl;
  } else {
    std::cout << "Saved free cell debug image to: " << file << std::endl;
  }
}
std::vector<nav_msgs::Path> CoveragePathPlanner::get_coverage_paths(const cv::Point2f& start,const int min_path) {
  using namespace std;

  bool use_bezier_optimization = true;

  coarse_path_.clear();
  fine_path_.clear();

  if (!initialized_) {
    ROS_ERROR("uninitialized!");
    return coarse_path_;
  }

  // auto start_secs = ros::Time::now();

  auto dist_threshold = AMPLIFY * resolution_ * model_size_cell_;
  dist_threshold = dist_threshold * dist_threshold;

  nav_msgs::Path path_pool;
  geometry_msgs::PoseStamped posestamped_pool;
  geometry_msgs::Pose pose_pool;

  std::optional<std::vector<CellIndex>> cells;
  cells = (clean_area_type_ == CleanAreaType::backshaped)
              ? get_backshaped_indexed_path(start)
              : get_zigzag_indexed_path(start);
  if (!cells)
    return coarse_path_;

  ROS_INFO("transform start.");
  /**transform**/
  auto size_y = rotated_costmap_->getSizeInCellsY();
  for (auto const &cell : cells.value()) {
    rotated_costmap_->mapToWorld(
        static_cast<uint32_t>(cell.col * model_size_cell_ +
                              cells_poses_[cell.row][cell.col].col),
        static_cast<uint32_t>(size_y -
                              (cell.row * model_size_cell_ +
                               cells_poses_[cell.row][cell.col].row) -
                              1),
        pose_pool.position.x, pose_pool.position.y);

    pose_pool.orientation.w = 1;
    pose_pool.orientation.x = 0;
    pose_pool.orientation.y = 0;
    pose_pool.orientation.z = 0;
    posestamped_pool.header.stamp = ros::Time::now();
    posestamped_pool.header.frame_id = "map";
    posestamped_pool.pose = pose_pool;

    if (!path_pool.poses.empty()) {
      double current_dist_sq = dist2(posestamped_pool, path_pool.poses.back());
      if (cell.source == PointSource::JUMP || current_dist_sq > dist_threshold) {
        // 只要是跳转点，或者超过距离阈值，强行断开
        coarse_path_.emplace_back(path_pool);
        path_pool.poses.clear();
        path_pool.poses.emplace_back(posestamped_pool);
        continue;
      }

      path_pool.poses.emplace_back(posestamped_pool);
    } else {
      path_pool.poses.emplace_back(posestamped_pool);
    }
  }

  coarse_path_.emplace_back(path_pool);
  
  
  ROS_INFO("transform finish.coarse_path_ size=%d",
           static_cast<int>(coarse_path_.size()));

#ifdef DEBUG_MODE
  ofstream f1(debug_path_ + "origin.txt", ios::trunc | ios::out);
  for (auto const &pose : planning_polygon_.poses) {
    f1 << pose.pose.position.x << "," << pose.pose.position.y << ","
       << pose.pose.orientation.z << endl;
  }
#endif
#ifdef DEBUG_MODE
#ifdef REVERSE_COARSE_PATH
  std::reverse(coarse_path_.begin(), coarse_path_.end());
  for (auto &path : coarse_path_) {
    std::reverse(path.poses.begin(), path.poses.end());
  }
#endif
#endif
  for (auto &path : coarse_path_) {
    auto iter = path.poses.begin();
    for (; iter < path.poses.end() - 1; iter++) {
      (*iter).pose.orientation.z =
          atan2((*(iter + 1)).pose.position.y - (*iter).pose.position.y,
                (*(iter + 1)).pose.position.x - (*iter).pose.position.x);
#ifdef DEBUG_MODE
      f1 << (*iter).pose.position.x << "," << (*iter).pose.position.y << ","
         << (*iter).pose.orientation.z << endl;
#endif
    }
    (*iter).pose.orientation.z = (*(iter - 1)).pose.orientation.z;
#ifdef DEBUG_MODE
    f1 << (*iter).pose.position.x << "," << (*iter).pose.position.y << ","
       << (*iter).pose.orientation.z << endl;
#endif
  }
#ifdef DEBUG_MODE
  f1.close();
#endif

  /**cut down way points**/
  ROS_INFO("cut down way points start...");
  for (auto &path : coarse_path_) {
    path.poses.back().pose.orientation.x = static_cast<double>(
        CoarsePathSeparate::break_point); // 每条路径的终点都应当标记为break
                                          // point
    auto path_length = path.poses.size();
    for (auto it = path.poses.begin() + 1; it < path.poses.end() - 1; it++) {
      //#348 opposite direction
      if (abs(abs((it - 1)->pose.orientation.z - it->pose.orientation.z) -
              M_PI) < ZERO &&
          it->pose.orientation.z == (it + 1)->pose.orientation.z) {
        (it - 1)->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::delete_point);
        (it + 1)->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::break_point);
      }

      if ((it - 1)->pose.orientation.z == it->pose.orientation.z &&
          it->pose.orientation.z == (it + 1)->pose.orientation.z &&
          it->pose.orientation.x !=
              static_cast<double>(CoarsePathSeparate::break_point)) { // SS
        it->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::delete_point);
      }

      if (it >= path.poses.begin() + 2 &&
          (it - 2)->pose.orientation.z != (it - 1)->pose.orientation.z &&
          (it - 1)->pose.orientation.z == it->pose.orientation.z &&
          it->pose.orientation.z == (it + 1)->pose.orientation.z) { // CSS
        it->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::break_point);
      }

      if (it >= path.poses.begin() + 2 &&
          (it - 2)->pose.orientation.z == (it - 1)->pose.orientation.z &&
          (it - 1)->pose.orientation.z == it->pose.orientation.z &&
          it->pose.orientation.z != (it + 1)->pose.orientation.z) { // SSC
        it->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::break_point);
      }

      if (path_length > 5 && it >= path.poses.begin() + 2 &&
          (it - 1)->pose.orientation.z != it->pose.orientation.z &&
          (it - 2)->pose.orientation.x !=
              static_cast<double>(CoarsePathSeparate::break_point)) {
        (it - 1)->pose.orientation.x =
            static_cast<double>(CoarsePathSeparate::break_point);
      }
    }
  }

  ROS_INFO("cut down way points finish");
  nav_msgs::Path path_temp;
  vector<nav_msgs::Path> paths_temp;
  for (auto &path : coarse_path_) {
    if( path.poses.size() < min_path )
        continue;    // 路径太短，丢弃

    for (auto &pose : path.poses) {
      if (pose.pose.orientation.x !=
          static_cast<double>(CoarsePathSeparate::delete_point)) {
        path_temp.poses.emplace_back(pose);
      }
      if (pose.pose.orientation.x ==
          static_cast<double>(CoarsePathSeparate::break_point)) {
        paths_temp.emplace_back(path_temp);
        path_temp.poses.clear();
        path_temp.poses.emplace_back(pose);
      }
    }
    paths_temp.emplace_back(path_temp);
    path_temp.poses.clear();
  }

  //	    auto cv_secs = ros::Time::now();
  //	    ROS_WARN("Get way points time is %.2f secs", (cv_secs -
  // start_secs).toSec());

#ifdef DEBUG_MODE
  ofstream f2(debug_path_ + "cutdown.txt", ios::trunc | ios::out);
  for (auto const &path : paths_temp) {
    for (auto const &pose : path.poses) {
      f2 << pose.pose.position.x << "," << pose.pose.position.y << ","
         << pose.pose.orientation.z << endl;
    }
  }
  f2.close();
#endif

  /**rotate back path**/
  if (angle_ != 0.0) {
#ifdef DEBUG_MODE
    ofstream f3(debug_path_ + "rotateback.txt", ios::trunc | ios::out);
#endif
    auto cos_an = cos(-angle_);
    auto sin_an = sin(-angle_);
    for (auto &path : paths_temp) {
      for (auto &pose : path.poses) {
        auto temp_x = pose.pose.position.x;
        auto temp_y = pose.pose.position.y;
        pose.pose.position.x = center_.x + (temp_x - center_.x) * cos_an -
                               (temp_y - center_.y) * sin_an;
        pose.pose.position.y = center_.y + (temp_x - center_.x) * sin_an +
                               (temp_y - center_.y) * cos_an;
#ifdef DEBUG_MODE
        f3 << pose.pose.position.x << "," << pose.pose.position.y << endl;
#endif
      }
    }
#ifdef DEBUG_MODE
    f3.close();
#endif
  }
  //	    auto rotate_secs = ros::Time::now();
  //	    ROS_WARN("Rotate way points time is %.2f secs", (rotate_secs -
  // cv_secs).toSec());

  /**optimize**/
  ROS_INFO("optimize path start...");
  vector<nav_msgs::Path> paths_opt_temp;
  for (size_t i = 0; i < paths_temp.size(); i++) {
    nav_msgs::Path om_path;
    if (use_bezier_optimization) {
      // 开启：走圆滑曲线
      om_path = get_bezier_optimized_path(paths_temp[i]);
    } else {
      // 关闭：走直线 (横平竖直)
      om_path = paths_temp[i];
    }

    if (om_path.poses.size() < 2)
      continue;
    if (i != paths_temp.size() - 1) {
      om_path.poses.pop_back();
    }

    paths_opt_temp.emplace_back(om_path);
  }
  ROS_INFO("optimize path finish");

  path_temp.poses.clear();

  bool border_contained = false;

  // nav_msgs::Path border;
  // bool border_contained;
  // if (save_border(border)) {
  //   ROS_INFO("Save border.");
  //   border_contained = true;
  //   fine_path_.emplace_back(border);
  // } else {
  //   ROS_WARN("Rect non free");
  //   border_contained = false;
  // }

  // adjust_border(paths_temp.front(), border);

  if (!paths_opt_temp.empty()) {
    auto iter = paths_opt_temp.begin();
    for (; iter < paths_opt_temp.end() - 1; iter++) {
      path_temp.poses.insert(path_temp.poses.end(), (*iter).poses.begin(),
                             (*iter).poses.end());
      if (dist2((*iter).poses.back(), (*(iter + 1)).poses.front()) > 0.1) {
        fine_path_.emplace_back(path_temp);
        path_temp.poses.clear();
      }
    }
    path_temp.poses.insert(path_temp.poses.end(), (*iter).poses.begin(),
                           (*iter).poses.end());
  }

  fine_path_.emplace_back(path_temp);
  
#ifdef DEBUG_MODE
  ofstream f4(debug_path_ + "optimize.txt", ios::trunc | ios::out);
  for (auto const &path : fine_path_) {
    for (auto const &pose : path.poses) {
      f4 << pose.pose.position.x << "," << pose.pose.position.y << endl;
    }
  }

  f4.close();
#endif
  auto iter = fine_path_.begin();
  if (border_contained)
    ++iter;
  for (; iter != fine_path_.end(); ++iter) {
    for (auto &pose : (*iter).poses) {
      if (pose.pose.orientation.x != 0) {
        pose.pose.orientation.x = 0;
      }
    }
  }
  //#479 work around: check abnormal points
  iter = fine_path_.begin();
  if (border_contained) {
    ++iter;
  }

  for (; iter != fine_path_.end(); ++iter) {
    for (auto it = (*iter).poses.begin(); it != (*iter).poses.end();) {
      if (abs((*it).pose.position.x) < ZERO &&
          abs((*it).pose.position.y) < ZERO &&
          abs((*it).pose.orientation.z) < ZERO &&
          abs((*it).pose.orientation.w - 1.0) < ZERO) {
        ROS_ERROR_STREAM("Found abnormal zero points!");
        it = (*iter).poses.erase(it);
      } else {
        ++it;
      }
    }
  }

  // auto optimize_secs = ros::Time::now();
  //	    ROS_WARN("Optimize way points time is %.2f secs", (optimize_secs -
  // rotate_secs).toSec());
  // ROS_WARN("Total time is %.2f secs", (optimize_secs - start_secs).toSec());
  ROS_INFO("Generate %lu paths.", fine_path_.size());

  return fine_path_;
}

std::vector<nav_msgs::Path>
CoveragePathPlanner::get_island_paths(const cv::Mat &binmat,
                 std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
                 const std::vector<cv::Point> &boundary,
                 double island_min_area,double island_path_shrink_dis,
                 const cv::Point& start_pt)
{
  std::vector<nav_msgs::Path> result;
  if (binmat.empty() || !costmap)
    return result;

  CV_Assert(binmat.type() == CV_8UC1);

  const int rows = binmat.rows;
  const int cols = binmat.cols;
  const double resolution = costmap->getResolution();

  // ----------------------------------------------------
  // Step 1: 提取障碍区域 (0)
  // ----------------------------------------------------
  cv::Mat obs_mask;
  cv::inRange(binmat, 0, 0, obs_mask); // 0 -> 255, 其他 -> 0

   // Step2:强制限制在 boundary 内 ---
  if (!boundary.empty()) {
      cv::Mat area_limit = cv::Mat::ones(rows, cols, CV_8UC1) * 255; // 默认全是障碍
      std::vector<std::vector<cv::Point>> contours_to_draw = {boundary};
      // 只有 boundary 内部设为 0 (非障碍)
      cv::fillPoly(area_limit, contours_to_draw, cv::Scalar(0));
      // 合并：原始障碍 + boundary 以外的区域
      obs_mask |= area_limit; 
  }

  // ----------------------------------------------------
  // Step 3: 连通域
  // ----------------------------------------------------
  cv::Mat labels, stats, centroids;
  int num = cv::connectedComponentsWithStats(
      obs_mask, labels, stats, centroids, 8, CV_32S);

  // ----------------------------------------------------
  // Step 3: 遍历每个障碍连通域
  // ----------------------------------------------------
  for (int i = 1; i < num; ++i) {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area_px = stats.at<int>(i, cv::CC_STAT_AREA);

    // ---------- 3.1 排除触边区域（外部 unknown） ----------
    if (x == 0 || y == 0 || (x + w) >= cols || (y + h) >= rows)
      continue;

    // ---------- 3.2 面积过滤 ----------
    double area_m2 = area_px * resolution * resolution;
    if (area_m2 < island_min_area )
      continue;

    // ---------- 3.3 提取该孤岛 mask ----------
    cv::Mat island_mask = (labels == i);
    island_mask.convertTo(island_mask, CV_8U, 255);

    // ----------------------------------------------------
    // Step 4: 查找该孤岛的外轮廓
    // ----------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
#if CV_MAJOR_VERSION <= 3
    cv::findContours(island_mask, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);
#else
    cv::findContours(island_mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);
#endif

    if (contours.empty())
      continue;

    // 取最大轮廓（理论上只有一个）
    size_t max_idx = 0;
    double max_area = cv::contourArea(contours[0]);
    for (size_t k = 1; k < contours.size(); ++k) {
      double a = cv::contourArea(contours[k]);
      if (a > max_area) {
        max_area = a;
        max_idx = k;
      }
    }

    std::vector<cv::Point>& contour = contours[max_idx];
    if (contour.size() < 2)
      continue;

    // --- Step 4.1: 重新排列轮廓起点 ---
    if (start_pt.x != 0 && start_pt.y != 0 ) {
      int best_idx = 0;
      double min_dist_sq = std::numeric_limits<double>::max();
      for (int k = 0; k < (int)contour.size(); ++k) {
        double dx = contour[k].x - start_pt.x;
        double dy = contour[k].y - start_pt.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < min_dist_sq) {
          min_dist_sq = d2;
          best_idx = k;
        }
      }
      // 将 best_idx 移动到首位
      std::rotate(contour.begin(), contour.begin() + best_idx, contour.end());
    }


    // ----------------------------------------------------
    // Step 5: 轮廓点 → world 坐标 → nav_msgs::Path
    // ----------------------------------------------------
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    auto size_y = rotated_costmap_->getSizeInCellsY();
    for (auto const &pt : contour) {
      unsigned int mx = static_cast<unsigned int>(pt.x);
      unsigned int my = static_cast<unsigned int>(pt.y);

      double wx, wy;
      costmap->mapToWorld(mx, size_y - my- 1,wx, wy);

      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;

      path.poses.emplace_back(ps);
    }
    // 闭合轮廓
    if (!path.poses.empty())
      path.poses.emplace_back(path.poses.front());
    

    // 确保存路径是顺时针
    if (path.poses.size() > 2) {
      bool ok=shrink_Path(island_path_shrink_dis,path);
      if(ok==false) ROS_ERROR("shrink_Path FAIL!");

      // 在世界坐标系中，area > 0 是逆时针，area < 0 是顺时针
      // 为了右沿边（顺时针），我们需要 area < 0
      double area = 0;
      for (size_t k = 0; k < path.poses.size() - 1; k++) {
          area += (path.poses[k].pose.position.x * path.poses[k+1].pose.position.y) -
                  (path.poses[k+1].pose.position.x * path.poses[k].pose.position.y);
      }
      
      ROS_INFO("area=%0.4f.",area);
      if (area > 0) {
        ROS_INFO("revers the island path.");
          std::reverse(path.poses.begin(), path.poses.end());
      }else
        ROS_INFO("not need revers the island path.");

        result.emplace_back(path);
    }
  }

  return result;
}
std::vector<nav_msgs::Path>
CoveragePathPlanner::get_island_paths_new(const cv::Mat &binmat,
                 std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
                 const std::vector<cv::Point> &boundary,
                 double island_min_area, double island_path_shrink_dis,
                 const cv::Point& start_pt)
{
    std::vector<nav_msgs::Path> result;
    if (binmat.empty() || !costmap)
        return result;

    CV_Assert(binmat.type() == CV_8UC1);

    const int rows = binmat.rows;
    const int cols = binmat.cols;
    const double resolution = costmap->getResolution();

    // 1. 提取原始全局障碍掩码 (obs_mask: 255为障碍, 0为可通行)
    cv::Mat obs_mask;
    cv::inRange(binmat, 0, 0, obs_mask); 

    if (!boundary.empty()) {
        cv::Mat area_limit = cv::Mat::ones(rows, cols, CV_8UC1) * 255;
        std::vector<std::vector<cv::Point>> contours_to_draw = {boundary};
        cv::fillPoly(area_limit, contours_to_draw, cv::Scalar(0));
        obs_mask |= area_limit; 
    }

    // 2. 连通域分析
    cv::Mat labels, stats, centroids;
    int num = cv::connectedComponentsWithStats(obs_mask, labels, stats, centroids, 8, CV_32S);

    // 计算膨胀所需的像素半径
    int kernel_size = std::max(1, static_cast<int>(std::round(island_path_shrink_dis / resolution)));
    // 创建圆形结构元素，使扩张更平滑
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1));

    for (int i = 1; i < num; ++i) {
        int area_px = stats.at<int>(i, cv::CC_STAT_AREA);
        double area_m2 = area_px * resolution * resolution;
        if (area_m2 < island_min_area) continue;

        // 排除触边区域
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (x <= 0 || y <= 0 || (x + w) >= cols || (y + h) >= rows) continue;

        // 3. 提取单个孤岛 Mask
        cv::Mat island_mask = (labels == i);
        island_mask.convertTo(island_mask, CV_8U, 255);

        // 4. 【核心步骤】使用膨胀进行路径扩张
        cv::Mat expanded_mask;
        cv::dilate(island_mask, expanded_mask, element);

        // 5. 提取膨胀后的外轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(expanded_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

        if (contours.empty()) continue;

        // 获取最大轮廓
        auto max_it = std::max_element(contours.begin(), contours.end(), 
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        std::vector<cv::Point> contour = *max_it;

        // 6. 路径点纠偏：确保膨胀后的点不落在其他障碍物上
        // for (auto &pt : contour) {
        //     // 如果膨胀后的点不幸进入了全局障碍区（比如两个孤岛靠得很近）
        //     if (obs_mask.at<uchar>(pt) == 255) {
        //         // 这里可以简单处理：保持原样（由局部规划器避障）
        //         // 或者利用原本的 island_mask 边缘点，这里采用直接保留
        //     }
        // }

        // 7. 重新排列起点
        if (start_pt.x != 0 || start_pt.y != 0) {
            auto min_it = std::min_element(contour.begin(), contour.end(), 
                [&](const cv::Point& a, const cv::Point& b) {
                    return cv::norm(a - start_pt) < cv::norm(b - start_pt);
                });
            std::rotate(contour.begin(), min_it, contour.end());
        }

        // 8. 转换为 nav_msgs::Path
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        auto size_y = costmap->getSizeInCellsY();

        for (const auto &pt : contour) {
            double wx, wy;
            // 注意：坐标映射逻辑需与你的 costmap 设置一致
            costmap->mapToWorld(pt.x, size_y - pt.y - 1, wx, wy);

            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = wx;
            ps.pose.position.y = wy;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }

        if (!path.poses.empty()) {
            path.poses.push_back(path.poses.front()); // 闭合
            
            // 9. 方向校验（确保顺时针/逆时针符合沿边需求）
            double area = 0;
            for (size_t k = 0; k < path.poses.size() - 1; k++) {
                area += (path.poses[k].pose.position.x * path.poses[k+1].pose.position.y) -
                        (path.poses[k+1].pose.position.x * path.poses[k].pose.position.y);
            }
            if (area > 0) std::reverse(path.poses.begin(), path.poses.end());

            result.push_back(path);
        }
    }
    return result;
}

nav_msgs::Path CoveragePathPlanner::shrink_polygon(const float shrink_dist,
    const std::vector<cv::Point> &in_ploy){
  clipper::ClipperOffset polygon_offset;
  clipper::Paths o_paths;
  clipper::Path i_path;
  i_path.clear();
  o_paths.clear();
 
  nav_msgs::Path new_path;
  new_path.header.frame_id = "map";
  new_path.header.stamp = ros::Time::now();

  int int_shrink_dist = static_cast<int>(shrink_dist * 1000);

  // 转成 clipper 坐标（放大1000倍）
  auto size_y = rotated_costmap_->getSizeInCellsY();
  for (int i = 0; i < in_ploy.size(); ++i) {
    double wx, wy;
    rotated_costmap_->mapToWorld(in_ploy[i].x, size_y - in_ploy[i].y- 1,wx, wy);
    clipper::IntPoint point(wx * 1000, wy * 1000);
    // 过滤太近的点
    if (!i_path.empty()) {
      clipper::IntPoint last = i_path.back();
      if (std::hypot(point.X - last.X, point.Y - last.Y) < 150)
        continue;
    }

    i_path.push_back(point);
  }

  // 保证闭合
  if (i_path.size() > 1 && i_path.front() != i_path.back()) {
    i_path.push_back(i_path.front());
  }

  /// 1. 简化轮廓
  clipper::Paths simple_paths;
  clipper::SimplifyPolygon(i_path, simple_paths);

  // 2. 对每个多边形做 offset
  for (const auto &p : simple_paths) {
    clipper::Path path = p;

    // -------- 关键：强制统一为 CCW --------
    if (!clipper::Orientation(path)) {
      std::reverse(path.begin(), path.end());
    }

    polygon_offset.Clear();

    // 使用圆角，避免拐角外推
    polygon_offset.AddPath(path,
                            clipper::jtRound,
                            clipper::etClosedPolygon);

    // -------- 关键：直接使用 shrink_dist 语义 --------
    // 你定义：
    // shrink_dist < 0 → 向内
    // shrink_dist > 0 → 向外
    int int_offset = static_cast<int>(shrink_dist * 1000);

    clipper::Paths offset_paths;
    polygon_offset.Execute(offset_paths, int_offset);

    o_paths.insert(o_paths.end(),
                  offset_paths.begin(),
                  offset_paths.end());
  }

  // 3. 找出面积最大的 polygon
  double max_area = 0;
  clipper::Path max_path;
  for (const auto &path : o_paths) {
    double area = std::abs(clipper::Area(path));
    if (area > max_area) {
      max_area = area;
      max_path = path;
    }
  }


  for (const auto &pt : max_path) {
    geometry_msgs::PoseStamped pose;
    double wx = pt.X / 1000;
    double wy = pt.Y / 1000;

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    new_path.poses.emplace_back(pose);
  }

  return new_path;
}
bool CoveragePathPlanner::shrink_Path(const float shrink_dist,nav_msgs::Path &path){
if (path.poses.size() < 3) return false;

  clipper::ClipperOffset polygon_offset;
  clipper::Paths o_paths;
  clipper::Path i_path;

  i_path.clear();
  o_paths.clear();

  // Clipper 使用整数坐标，放大倍数
  const double scale = 1000.0;
  int int_shrink_dist = static_cast<int>(shrink_dist * scale);

  // 1. Path(map) -> world -> clipper
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double wx = path.poses[i].pose.position.x;
    double wy = path.poses[i].pose.position.y;

    clipper::IntPoint point(static_cast<int>(wx * scale),
                            static_cast<int>(wy * scale));

    // 过滤太近的点（防止Clipper自交）
    if (!i_path.empty()) {
      clipper::IntPoint last = i_path.back();
      if (std::hypot(double(point.X - last.X),
                     double(point.Y - last.Y)) < 0.15 * scale) {
        continue;
      }
    }

    i_path.push_back(point);
  }

  // 2. 保证闭合
  if (i_path.size() > 1 && i_path.front() != i_path.back()) {
    i_path.push_back(i_path.front());
  }

  if (i_path.size() < 4) return false;

  // 3. 简化多边形，去除自交
  clipper::Paths simple_paths;
  clipper::SimplifyPolygon(i_path, simple_paths);

  // 4. 对每个简单多边形做 offset
  for (const auto &p : simple_paths) {
    polygon_offset.Clear();
    polygon_offset.AddPath(p, clipper::jtMiter, clipper::etClosedPolygon);

    clipper::Paths offset_paths;
    polygon_offset.Execute(offset_paths, int_shrink_dist);

    o_paths.insert(o_paths.end(), offset_paths.begin(), offset_paths.end());
  }

  if (o_paths.empty()) return false;

  // 5. 选取面积最大的结果（防止碎片）
  double max_area = 0;
  clipper::Path max_path;
  for (const auto &p : o_paths) {
    double area = std::abs(clipper::Area(p));
    if (area > max_area) {
      max_area = area;
      max_path = p;
    }
  }

  if (max_path.size() < 3) return false;

  // 6. Clipper -> world -> Path
  nav_msgs::Path new_path;
  new_path.header = path.header;

  for (const auto &pt : max_path) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;

    double wx = pt.X / scale;
    double wy = pt.Y / scale;

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    new_path.poses.emplace_back(pose);
  }

  // 7. 回写
  path = new_path;
  return true;
}
void CoveragePathPlanner::adjust_border(nav_msgs::Path const &cover,
                                        nav_msgs::Path &border) {
  using namespace std;

  auto dist2_min = DBL_MAX;
  size_t idx = 0;
  for (size_t i = 0; i < border.poses.size(); ++i) {
    auto dist2_tmp = dist2(border_polygon_.poses[i], cover.poses.front());
    if (dist2_tmp < dist2_min) {
      dist2_min = dist2_tmp;
      idx = i;
    }
  }

  if (idx != 0) {
    vector<geometry_msgs::PoseStamped> fb, bb;
    fb.insert(fb.begin(), border.poses.begin(),
              border.poses.begin() + static_cast<long>(idx));
    bb.insert(bb.begin(), border.poses.begin() + static_cast<long>(idx),
              border.poses.end());
    border.poses.clear();
    border.poses.insert(border.poses.end(), bb.begin(), bb.end());
    border.poses.insert(border.poses.end(), fb.begin(), fb.end());
  }
}
bool CoveragePathPlanner::save_border(nav_msgs::Path &border) {
  if (border_polygon_.poses.size() < 2)
    return false;
  std::vector<nav_msgs::Path> border_tmp;
  for (size_t i = 0; i < border_polygon_.poses.size(); ++i) {
    nav_msgs::Path path_tmp;
    path_tmp.poses.emplace_back(border_polygon_.poses[i]);
    if (i == border_polygon_.poses.size() - 1) {
      path_tmp.poses.emplace_back(border_polygon_.poses.front());
    } else {
      path_tmp.poses.emplace_back(border_polygon_.poses[i + 1]);
    }
    border_tmp.emplace_back(path_tmp);
  }

  for (auto const &tmp : border_tmp) {
    auto om_path = get_bezier_optimized_path(tmp);
    om_path.poses.pop_back();
    border.poses.insert(border.poses.end(), om_path.poses.begin(),
                        om_path.poses.end());
  }

  if (border_polygon_.poses.size() > 4)
    return true;

  int non_free = 0;
  for (auto const &pose : border.poses) {
    auto x = pose.pose.position.x;
    auto y = pose.pose.position.y;
    unsigned mx, my;
    if (!rotated_costmap_->worldToMap(x, y, mx, my)) {
      if (++non_free > 3)
        return false;
      continue;
    }

    if (rotated_costmap_->getCost(mx, my) >=
        costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (++non_free > 3)
        return false;
    }
  }

  return true;
}
void CoveragePathPlanner::show_path(const cv::Mat &map,
                                    const std::vector<nav_msgs::Path> &paths,
                                    const std::string &name) {

  if (false == debug_)
    return;
  //
  cv::Mat canvas = costmat2Colormat(map);
  // 颜色列表（循环用）

  int color_idx = 0;
  int total_colors = static_cast<int>(paths.size());

  // 遍历每条 Path
  for (size_t k = 0; k < paths.size(); ++k) {
    const auto &path = paths[k];
    if (path.poses.size() < 2)
      continue;

    cv::Scalar color = colorFromIndex(color_idx, total_colors);
    color_idx++;

    // 4. 遍历路径点
    for (size_t i = 0; i < path.poses.size() - 1; ++i) {

      // 世界坐标变像素坐标：假设 map 是已经 rotate 回来的世界坐标图，
      // 若你有 map->world 的转换，请替换以下 x,y 的计算逻辑
      double x1 = path.poses[i].pose.position.x;
      double y1 = path.poses[i].pose.position.y;
      double x2 = path.poses[i + 1].pose.position.x;
      double y2 = path.poses[i + 1].pose.position.y;

      unsigned int px1, py1, px2, py2;
      int y_flip;
      rotated_costmap_->worldToMap(x1, y1, px1, py1);
      py1 = canvas.rows - py1 - 1;
      rotated_costmap_->worldToMap(x2, y2, px2, py2);
      py2 = canvas.rows - py2 - 1;

      // 安全检查
      if (px1 < 0 || py1 < 0 || px1 >= canvas.cols || py1 >= canvas.rows)
        continue;
      if (px2 < 0 || py2 < 0 || px2 >= canvas.cols || py2 >= canvas.rows)
        continue;

      // 画线
      cv::line(canvas, cv::Point(px1, py1), cv::Point(px2, py2), color, 2,
               cv::LINE_AA);

      // // 画节点
      // cv::circle(canvas, cv::Point(px1, py1), 2, color, -1);
    }
  }
  std::string save_path = "/home/pi/tmp/" + name;
  cv::imwrite(save_path, canvas);
}
void CoveragePathPlanner::show_astart_path(const cv::Mat &map,
                                           const std::vector<cv::Point> &points,
                                           int idx) {
  if (false == debug_)
    return;
  // 1. 基本检查
  if (map.empty() || points.empty()) {
    ROS_WARN("show_astart_path: empty map or points.");
    return;
  }

  // 2. 确保 map 是单通道，否则不能使用 COLOR_GRAY2BGR
  if (map.channels() != 1) {
    ROS_WARN("show_astart_path: input map must be single channel.");
    return;
  }

  // 3. 转为三通道显示用图
  cv::Mat debug;
  cv::cvtColor(map, debug, cv::COLOR_GRAY2BGR);

  // 4. 准备颜色表
  static const std::vector<cv::Scalar> kColorTable{
      cv::Scalar(0, 0, 255),     // 红
      cv::Scalar(0, 255, 0),     // 绿
      cv::Scalar(255, 0, 0),     // 蓝
      cv::Scalar(0, 255, 255),   // 黄
      cv::Scalar(255, 0, 255),   // 紫
      cv::Scalar(255, 255, 0),   // 青
      cv::Scalar(180, 105, 255), // 粉
      cv::Scalar(0, 128, 255)    // 橙
  };

  const cv::Scalar color = kColorTable[idx % kColorTable.size()];

  // 5. 绘制路径
  for (size_t i = 0; i < points.size() - 1; ++i) {
    cv::line(debug, points.at(i), points.at(i + 1), color, 1, cv::LINE_AA);
  }

  // 6. 构建保存路径
  std::string save_path = "/home/pi/tmp/astart_" + std::to_string(idx) + ".png";
  cv::imwrite(save_path, debug);
  ROS_INFO_STREAM("Saved A* path image: " << save_path);
}
void CoveragePathPlanner::show_miss_path(const cv::Mat &map,
                                         const cv::Point &start,
                                         const cv::Point &end, const int idx,
                                         const nav_msgs::Path &path,
                                        bool is_island) {
  if (false == debug_)
    return;
  // 1. 基本检查
  if (map.empty()) {
    ROS_WARN("show_astart_path: empty map or points.");
    return;
  }
  // 2. 转为三通道显示用图
  cv::Mat debug;
  cv::cvtColor(map, debug, cv::COLOR_GRAY2BGR);

  cv::circle(debug, start, 2, cv::Scalar(255, 0, 0), -1);
  cv::circle(debug, end, 2, cv::Scalar(255, 0, 255), -1);
  
  std::string save_path;
  if(is_island)
    save_path = "/home/pi/tmp/miss_" + std::to_string(idx) + ".png";
  else
    save_path = "/home/pi/tmp/island_miss_" + std::to_string(idx) + ".png";
  cv::imwrite(save_path, debug);
}
cv::Scalar CoveragePathPlanner::colorFromIndex(int idx, int total) {
  int hue = (idx * 180 / total) % 180;
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 200, 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(c[0], c[1], c[2]); // BGR
}
std::optional<std::vector<CellIndex>>
CoveragePathPlanner::get_zigzag_indexed_path(const cv::Point2f& start) {
  if (free_space_.empty()) {
    ROS_ERROR("No free space!");
    return std::nullopt;
  }

  if (planning_polygon_.poses.empty()) {
    ROS_ERROR("No zone peak!");
    return std::nullopt;
  }

  auto ret_tuple = get_start_point(start,planning_polygon_);
  if (!ret_tuple) {
    ROS_ERROR("Failed to set initial pose in map!");
    return std::nullopt;
  }

  using namespace std;

  std::vector<CellIndex> indexed_path;
  CellIndex init_point, next_point, cur_point;
  init_point.theta = 0;

  auto wx = std::get<0>(ret_tuple.value());
  auto wy = std::get<1>(ret_tuple.value());
  auto mx = std::get<2>(ret_tuple.value());
  auto my = std::get<3>(ret_tuple.value());
  ROS_INFO("start inittheta is %.2f, start point is [%.4f, %.4f]",
           init_point.theta, wx, wy);

  change_weight_tiny(my);
  init_point.row = std::get<4>(ret_tuple.value()).row;
  init_point.col = std::get<4>(ret_tuple.value()).col;
  cur_point = init_point;
  //处理起点在右侧或者右侧有障碍物的情况
  cur_point.theta =
      (cur_point.col == neural_costmat_.cols - 1 ||
       neural_costmat_.at<float>(cur_point.row, cur_point.col + 1) == -FLT_MAX)
          ? 180
          : 0;
  indexed_path.emplace_back(init_point);
  //将第一个点插入indexed_path之后，就从遍历集中删除
  auto free_space = free_space_;
  auto init_point_ptr =
      find_if(free_space.begin(), free_space.end(), [&](CellIndex const &cell) {
        return cell.col == init_point.col && cell.row == init_point.row;
      });
  if (init_point_ptr != free_space.end()) {
    free_space.erase(init_point_ptr);
  }

  float e = 0;
  float v = 0.0;
  float delta_theta = 0;
  vector<float> thetaVec = {0, 180};
  /**main planning loop**/
  auto bound_of_free_space = FreeSpaceCalculator::find_bound_of_free_space(
      free_space_, neural_costmat_);
  auto min_row_of_free_space = std::get<1>(bound_of_free_space);
  auto min_col_of_free_space = std::get<0>(bound_of_free_space);
  for (int loop = 0; loop < LOOP_MAX; ++loop) {
    auto maxIndex = 0UL;
    float max_v = OUT_SCORE - 1;
    neural_costmat_.at<float>(cur_point.row, cur_point.col) = OUT_SCORE;
    auto th = static_cast<float>(cur_point.theta);
    for (size_t id = 0; id < thetaVec.size(); ++id) {
      delta_theta = std::abs(thetaVec[id] - th);
      if (delta_theta > 180) {
        delta_theta = 360 - delta_theta;
      }

      e = 1 - abs(delta_theta) / 180;
      switch (id) {
      case 0:
        if (cur_point.col == neural_costmat_.cols - 1) {
          v = -FLT_MAX;
          break;
        }

        v = neural_costmat_.at<float>(cur_point.row, cur_point.col + 1) +
            C_0 * e;
        break;
      case 1:
        if (cur_point.col == 0) {
          v = -FLT_MAX;
          break;
        }

        v = neural_costmat_.at<float>(cur_point.row, cur_point.col - 1) +
            C_0 * e + weight_tiny_;
        break;
      default:
        break;
      }

      if (v > max_v || (v == max_v && id > maxIndex)) {
        max_v = v;
        maxIndex = id;
      }
    }

    if (free_space.empty()) {
      ROS_WARN("The program has gone through %lu steps", indexed_path.size());
      return indexed_path;
    }

    if (max_v <= 0) {
      float min_dist = FLT_MAX;
      float dist;
      int ii = 0;
      int min_index = -1;
      for (auto const &fs : free_space_) {
        if (neural_costmat_.at<float>(fs.row, fs.col) > 0) {
          dist = static_cast<float>(
              (cur_point.row - fs.row) * (cur_point.row - fs.row) +
              (cur_point.col - fs.col) * (cur_point.col - fs.col));
          if (dist < min_dist) {
            min_dist = dist;
            min_index = ii;
          }
        }

        ++ii;
      }

      if (min_index == -1) {
        ROS_WARN("The program has gone through %lu steps", indexed_path.size());
        break;
      }

      next_point = free_space_[static_cast<unsigned long>(min_index)];
      next_point.theta = atan2(next_point.col - cur_point.col,
                               next_point.row - cur_point.row) /
                         M_PI * 180;
      if (next_point.theta < 0) {
        next_point.theta = 360 + next_point.theta;
      }

      // ROS_DEBUG_STREAM("next point index: " << min_index << " " << "distance:
      // " << std::sqrt(min_dist)); ROS_DEBUG_STREAM("current point: " <<
      // cur_point.row << ", " << cur_point.col); ROS_DEBUG_STREAM("next point:
      // " << next_point.row << ", " << next_point.col);
      cur_point = next_point;
      indexed_path.emplace_back(next_point);

      continue;
    }

    /**next point**/
    switch (maxIndex) {
    case 0:
      next_point.row = cur_point.row;
      next_point.col = cur_point.col + 1;
      break;
    case 1:
      next_point.row = cur_point.row;
      next_point.col = cur_point.col - 1;
      break;
    default:
      break;
    }

    // ROS_INFO("in same line");
    // ROS_INFO_STREAM("current point: " << cur_point.row << ", "
    //                                   << cur_point.col);
    // ROS_INFO_STREAM("next point: " << next_point.row << ", " <<
    // next_point.col);
    next_point.theta = thetaVec[maxIndex];
    cur_point = next_point;
    indexed_path.emplace_back(next_point);
    auto next_ptr = find_if(
        free_space.begin(), free_space.end(), [&](CellIndex const &cell) {
          return cell.row == next_point.row && cell.col == next_point.col;
        });
    if (next_ptr != free_space.end()) {
      free_space.erase(next_ptr);
    }
  }

  ROS_WARN("CoveragePathPlanner: in function get_zigzag_indexed_path, loop "
           "reaches max(%d)",
           LOOP_MAX);
  return indexed_path;
}

std::optional<std::vector<CellIndex>>
CoveragePathPlanner::get_backshaped_indexed_path(const cv::Point2f& start) {
  if (free_space_.empty()) {
    ROS_ERROR("No free space!");
    return std::nullopt;
  }

  if (planning_polygon_.poses.empty()) {
    ROS_ERROR("No zone peak!");
    return std::nullopt;
  }

  auto ret_tuple = get_start_point(start, planning_polygon_);
  if (!ret_tuple) {
    ROS_ERROR("Failed to set initial pose in map!");
    return std::nullopt;
  }

  std::vector<CellIndex> indexed_path;
  CellIndex next_point, cur_point;

  auto wx = std::get<0>(ret_tuple.value());
  auto wy = std::get<1>(ret_tuple.value());
  auto mx = std::get<2>(ret_tuple.value());
  auto my = std::get<3>(ret_tuple.value());
  change_weight_tiny(my);

  // 基础方向定义（必须严格按顺时针顺序 0-7 排列）
  // 0:右, 1:右下, 2:下, 3:左下, 4:左, 5:左上, 6:上, 7:右上
  using dir_t = std::tuple<int, int, int>;
  const std::vector<dir_t> directs{
      std::make_tuple(0, 0, 1),  std::make_tuple(1, 1, 1),
      std::make_tuple(2, 1, 0),  std::make_tuple(3, 1, -1),
      std::make_tuple(4, 0, -1), std::make_tuple(5, -1, -1),
      std::make_tuple(6, -1, 0), std::make_tuple(7, -1, 1)};

  // --- 逆时针核心逻辑 ---
  // false 为逆时针，true 为顺时针
  bool target_clockwise = false; 

  // 初始化起点网格
  cur_point.row = scale_costmat_.rows - static_cast<int32_t>(my) / model_size_cell_ - 1;
  cur_point.col = static_cast<int32_t>(mx) / model_size_cell_;
  
  // 初始朝向：逆时针通常初始向右 (0)
  int current_theta_idx = 0; 
  cur_point.theta = current_theta_idx * 45;
  indexed_path.emplace_back(cur_point);
  
  ROS_INFO("Target Counter-Clockwise: %s, Start point: [%.4f, %.4f]", 
           target_clockwise ? "No" : "Yes", wx, wy);

  while (ros::ok()) {
    // 标记当前点已访问
    neural_costmat_.at<float>(cur_point.row, cur_point.col) = OUT_SCORE;

    // 定义搜索优先级偏移量
    // 逆时针（target_clockwise=false）：优先看右边 (idx + 2)，这样会沿着左侧边界绕行
    // 顺时针（target_clockwise=true）：优先看左边 (idx - 2)，这样会沿着右侧边界绕行
    std::vector<int> search_offsets;
    if (target_clockwise) {
        search_offsets = {-2, -1, 0, 1, 2, 3, 4, 3}; // 顺时针偏好
    } else {
        search_offsets = {2, 1, 0, -1, -2, -3, 4, 3}; // 逆时针偏好
    }

    bool find_near = false;
    for (int offset : search_offsets) {
      // 计算目标的绝对方向索引 (0-7循环)
      int dir_idx = (current_theta_idx + offset + 8) % 8;
      const auto& dir = directs[dir_idx];

      int r = cur_point.row + std::get<1>(dir);
      int c = cur_point.col + std::get<2>(dir);

      // 边界与障碍物检查
      if (r >= 0 && r < neural_costmat_.rows && c >= 0 && c < neural_costmat_.cols &&
          neural_costmat_.at<float>(r, c) > 0) {
        
        find_near = true;
        next_point.row = r;
        next_point.col = c;
        next_point.theta = std::get<0>(dir) * 45;
        next_point.source = PointSource::NEIGHBOR;

        indexed_path.emplace_back(next_point);
        cur_point = next_point;
        current_theta_idx = std::get<0>(dir); // 更新当前朝向索引
        break;
      }
    }

    if (find_near) continue;

    // --- 孤岛跳转逻辑 (当四周都没有路时) ---
    float min_dist = FLT_MAX;
    int min_index = -1;
    for (int ii = 0; ii < (int)free_space_.size(); ++ii) {
      auto const &fs = free_space_[ii];
      if (neural_costmat_.at<float>(fs.row, fs.col) > 0) {
        float dist = static_cast<float>((cur_point.row - fs.row) * (cur_point.row - fs.row) +
                                       (cur_point.col - fs.col) * (cur_point.col - fs.col));
        if (dist < min_dist) {
          min_dist = dist;
          min_index = ii;
        }
      }
    }

    if (min_index == -1) break; // 全部找完了

    // 执行跳转
    next_point = free_space_[static_cast<size_t>(min_index)];
    // 重新计算跳转后的朝向
    float angle = atan2(next_point.col - cur_point.col, next_point.row - cur_point.row) / M_PI * 180;
    if (angle < 0) angle += 360;
    next_point.theta = angle;
    next_point.source = PointSource::JUMP;
    cur_point = next_point;
    indexed_path.emplace_back(next_point);
    
    // 跳转后将朝向索引映射回 0-7
    current_theta_idx = static_cast<int>(std::round(angle / 45.0)) % 8;
  }

  ROS_INFO("get_backshaped_indexed_path finish! size=%d", static_cast<int>(indexed_path.size()));
  visualize_backshaped_path(indexed_path);
  return indexed_path;
}

nav_msgs::Path CoveragePathPlanner::get_bezier_optimized_path(
    nav_msgs::Path const &path) const {
  BezierTrajectoryGeneratorWaypoint bezier_traj_gen_way_points(PATH_STEP);
  return bezier_traj_gen_way_points.calculate_optimized_path(path);
}
void CoveragePathPlanner::change_weight_tiny(uint32_t first_y) {
  // renew weight_tiny_
  uint32_t mx_poly_center, my_poly_center;
  rotated_costmap_->worldToMap(polygon_center_.x, polygon_center_.y,
                               mx_poly_center, my_poly_center);
  weight_tiny_ = my_poly_center > first_y ? WEIGHT_TINY : -WEIGHT_TINY;
#ifdef DEBUG_MODE
  ROS_ERROR("y of planning_polygon_ in rotated_costmap_ = %u", first_y);
  ROS_ERROR("center of polygon in rotated_costmap_ = [%u, %u]", mx_poly_center,
            my_poly_center);
#endif
}
// std::optional<std::tuple<double, double, uint32_t, uint32_t, CellIndex>>
// CoveragePathPlanner::get_start_point(const cv::Point& start,nav_msgs::Path const &polygon) {
//   size_t min_index = 0;
//   auto x = planning_polygon_.poses[min_index].pose.position.x;
//   auto y = planning_polygon_.poses[min_index].pose.position.y;
//   unsigned mx, my;
//   if (!rotated_costmap_->worldToMap(x, y, mx, my)) {
//     return std::nullopt;
//   }

//   auto r0 =
//       scale_costmat_.rows - static_cast<int32_t>(my) / model_size_cell_ - 1;
//   auto c0 = static_cast<int32_t>(mx) / model_size_cell_;
//   int min_dist = INT_MAX;
//   auto r_min = 0;
//   auto c_min = 0;
//   for (auto const &cell : free_space_) {
//     if ((abs(cell.row - r0) + abs(cell.col - c0)) < min_dist) {
//       min_dist = abs(cell.row - r0) + abs(cell.col - c0);
//       r_min = cell.row;
//       c_min = cell.col;
//     }
//   }

//   mx = static_cast<uint32_t>(c_min * model_size_cell_ + model_size_cell_ / 2);
//   my = static_cast<uint32_t>((scale_costmat_.rows - r_min - 1) *
//                                  model_size_cell_ +
//                              model_size_cell_ / 2);
//   rotated_costmap_->mapToWorld(mx, my, x, y);
//   return std::make_tuple(x, y, mx, my, CellIndex{r_min, c_min, 0});
// }
std::optional<std::tuple<double, double, uint32_t, uint32_t, CellIndex>>
CoveragePathPlanner::get_start_point(const cv::Point2f & start,
                                     nav_msgs::Path const &polygon)
{
  if (rotated_costmap_ == nullptr)
    return std::nullopt;

  if (polygon.poses.empty())
    return std::nullopt;

  // ---------- Step 0: 选择参考点 ----------
  cv::Point2f ref_pt;
  if (start.x == 0.0 && start.y == 0.0) {
    ref_pt.x = polygon.poses[0].pose.position.x;
    ref_pt.y = polygon.poses[0].pose.position.y;
  } else {
    ref_pt = start;   // 直接是 world 坐标
  }

  // ---------- Step 1: 判断是否在 polygon 内 ----------
  std::vector<cv::Point2f> poly_pts;
  for (auto const &p : polygon.poses) {
    poly_pts.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  bool inside = cv::pointPolygonTest(poly_pts, ref_pt, false) >= 0;

  // ---------- Step 2: 若不在 polygon 中，找最近边界点 ----------
  cv::Point2f query_pt = ref_pt;
  if (!inside) {
    double min_dist = std::numeric_limits<double>::max();
    cv::Point2f closest_pt = ref_pt;

    for (size_t i = 0; i < poly_pts.size(); ++i) {
      auto p1 = poly_pts[i];
      auto p2 = poly_pts[(i + 1) % poly_pts.size()];

      cv::Point2f v = p2 - p1;
      cv::Point2f w = ref_pt - p1;

      double c1 = v.dot(w);
      if (c1 <= 0) {
        double d = cv::norm(ref_pt - p1);
        if (d < min_dist) { min_dist = d; closest_pt = p1; }
        continue;
      }

      double c2 = v.dot(v);
      if (c2 <= c1) {
        double d = cv::norm(ref_pt - p2);
        if (d < min_dist) { min_dist = d; closest_pt = p2; }
        continue;
      }

      double b = c1 / c2;
      cv::Point2f pb = p1 + b * v;
      double d = cv::norm(ref_pt - pb);
      if (d < min_dist) { min_dist = d; closest_pt = pb; }
    }

    query_pt = closest_pt;
  }

  // ---------- Step 3: world → map ----------
  unsigned mx, my;
  if (!rotated_costmap_->worldToMap(query_pt.x, query_pt.y, mx, my)) {
    return std::nullopt;
  }

  // ---------- Step 4: 映射到缩放栅格 ----------
  auto r0 = scale_costmat_.rows - static_cast<int32_t>(my) / model_size_cell_ - 1;
  auto c0 = static_cast<int32_t>(mx) / model_size_cell_;

  int min_dist = INT_MAX;
  int r_min = 0, c_min = 0;
  for (auto const &cell : free_space_) {
    int d = abs(cell.row - r0) + abs(cell.col - c0);
    if (d < min_dist) {
      min_dist = d;
      r_min = cell.row;
      c_min = cell.col;
    }
  }

  // ---------- Step 5: cell → map → world ----------
  mx = static_cast<uint32_t>(c_min * model_size_cell_ + model_size_cell_ / 2);
  my = static_cast<uint32_t>((scale_costmat_.rows - r_min - 1) *
                                 model_size_cell_ +
                             model_size_cell_ / 2);

  double x, y;
  rotated_costmap_->mapToWorld(mx, my, x, y);

  return std::make_tuple(x, y, mx, my, CellIndex{r_min, c_min, 0});
}

bool CoveragePathPlanner::is_clockwise(nav_msgs::Path const &path) {
  auto n = path.poses.size();
  if (n < 3)
    return false;

  int i, j, k;
  int count = 0;
  double z;
  for (i = 0; i < n; ++i) {
    j = (i + 1) % n;
    k = (i + 2) % n;
    z = (path.poses[j].pose.position.x - path.poses[i].pose.position.x) *
            (path.poses[k].pose.position.y - path.poses[j].pose.position.y) -
        (path.poses[j].pose.position.y - path.poses[i].pose.position.y) *
            (path.poses[k].pose.position.x - path.poses[j].pose.position.x);
    if (z < 0) {
      --count;
    } else if (z > 0) {
      ++count;
    }
  }

  ROS_WARN("path polygon is %s", count < 0 ? "closewise" : "anticlosewise");
  return count < 0;
}
void CoveragePathPlanner::show_result_path(const cv::Mat &map,
                                           const nav_msgs::Path &path,
                                           const std::string &name,
                                           bool is_island) {
  if (false == debug_)
    return;
  
  if( path.poses.size()<2 ) return ;

  cv::Mat canvas = costmat2Colormat(map);

  cv::Scalar color;
  cv::Scalar color_raw_path = cv::Scalar(0, 0, 255);
  cv::Scalar color_astart = cv::Scalar(125, 255, 0);
  cv::Scalar color_start = cv::Scalar(255, 255, 0);

  // cv::namedWindow(name, cv::WINDOW_NORMAL);
  // cv::resizeWindow(name, 800, 600);

  for (size_t i = 0; i < path.poses.size() - 1; ++i) {

    // 世界坐标变像素坐标：假设 map 是已经 rotate 回来的世界坐标图，
    // 若你有 map->world 的转换，请替换以下 x,y 的计算逻辑
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i + 1].pose.position.x;
    double y2 = path.poses[i + 1].pose.position.y;
    double z1 = path.poses[i].pose.position.z;

    unsigned int px1, py1, px2, py2;
    int y_flip;
    rotated_costmap_->worldToMap(x1, y1, px1, py1);
    py1 = canvas.rows - py1 - 1;
    rotated_costmap_->worldToMap(x2, y2, px2, py2);
    py2 = canvas.rows - py2 - 1;

    // 安全检查
    if (px1 < 0 || py1 < 0 || px1 >= canvas.cols || py1 >= canvas.rows)
      continue;
    if (px2 < 0 || py2 < 0 || px2 >= canvas.cols || py2 >= canvas.rows)
      continue;

    if (z1 > 0.005)
      color = color_astart;
    else
      color = color_raw_path;
    // 画线
    cv::line(canvas, cv::Point(px1, py1), cv::Point(px2, py2), color, 3,
             cv::LINE_AA);

    // 画节点
    if (i == 0)
      cv::circle(canvas, cv::Point(px1, py1), 4, color_start, -1);

    // cv::imshow(name, canvas);
    // cv::waitKey(10);
  }

  std::string save_path;
  if(is_island)
    save_path = "/home/pi/tmp/s99_island_" + name;
  else
    save_path = "/home/pi/tmp/s99_coverage_" + name;

  cv::imwrite(save_path, canvas);
}
void CoveragePathPlanner::visualize_backshaped_path(
    const std::vector<CellIndex> &cells) {
  if (false == debug_)
    return;
  cv::Mat img;
  cv::cvtColor(work_binmat_, img, cv::COLOR_GRAY2BGR);
  // cv::namedWindow("Backshaped Indexed Path", cv::WINDOW_NORMAL);
  // cv::resizeWindow("Backshaped Indexed Path", 800, 600);
  for (size_t i = 1; i < cells.size(); ++i) {
    cv::Point p1(cells[i - 1].col * model_size_cell_,
                 cells[i - 1].row * model_size_cell_);

    cv::Point p2(cells[i].col * model_size_cell_,
                 cells[i].row * model_size_cell_);

    cv::line(img, p1, p2, cv::Scalar(0, 0, 255), 1);
    cv::circle(img, p2, 2, cv::Scalar(255, 0, 0), -1);

    // cv::imshow("Backshaped Indexed Path", img);
    // cv::waitKey(30);
  }
  std::string filename = "/home/pi/tmp/s9_backshaped_path.png";
  cv::imwrite(filename, img);
}

} // namespace ikitbot::planning
