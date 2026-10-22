//
// Created by tony on 2022/10/25.
//

#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "A_star_pathplanner.h"
#include "A_star_pathplanner_fast.h"
#include "BezierTrajectoryGeneratorWaypoint.h"
#include "FreeSpaceCalculator.h"


#include <robot_msg/CoveragePathPlan.h>

#include "clean_region.h"
#include "curve.h"
#include "json.h"
#include "yaml_helper.h"

namespace ikitbot::planning {
//    #define DEBUG_MODE
//#define REVERSE_COARSE_PATH

static constexpr int LOOP_MAX = 9000;
static constexpr long CHECK_MAX = 20;
static constexpr double ZERO = 1E-5;
static constexpr double AMPLIFY = 2.0;
static constexpr double CHECK_DIST = 2;
static constexpr double FIL_TOLER = 0.5;
static constexpr float C_0 = 50;
static constexpr float OUT_SCORE = -250;
static constexpr float WEIGHT_TINY = -1;
static constexpr double PATH_STEP = 0.05;

class CoveragePathPlanner {
  enum class CoarsePathSeparate : int {
    delete_point = 1,
    break_point = 2
  }; // 1:直线中点，需要删除，2:间断点，该点前后的点分属于不同组

  enum class CleanAreaType : uint8_t {
    zigzag = 0,
    backshaped = 1
  }; // 0:普通覆盖，1：车位覆盖，2：回字覆盖
public:
  CoveragePathPlanner(){
    rotated_costmap_=nullptr;
  };
  ~CoveragePathPlanner() = default;

  std::vector<nav_msgs::Path>
  get_path_interface(const cv::Mat &costmat,const cv::Point2f& start,
                     std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
                     const CleanRegionInfo &cr_info,bool has_island_path,
                     double island_min_area,double island_path_shrink_dis,
                     const int min_path,const int obs_inflate,bool debug);

private:
  void mapToWorld(const unsigned int mx, const unsigned int my, double &wx, double &wy);
  void worldToMap(const double wx, const double wy, unsigned int &mx, unsigned int &my);
  cv::Mat costmat2Colormat(const cv::Mat &costmat);
  cv::Mat costmat2Binmat(const cv::Mat &costmat);
 
  void ini_task(const cv::Mat &costmat,
    std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
    const CleanRegionInfo &cr_info);
  nav_msgs::Path get_merged_path(const std::vector<nav_msgs::Path>& raw_paths,bool is_island=false);
  nav_msgs::Path get_merged_smooth_path(const std::vector<nav_msgs::Path>& raw_paths,bool is_island=false);

  void show_cost_plygon(const cv::Mat costmat,const std::vector<cv::Point> &vertices,const std::string& tile);
  void tarnCVPath2RosPath(const std::vector<cv::Point>& points,nav_msgs::Path& path);

private:
  inline double dist2(geometry_msgs::PoseStamped const &p1,
                    geometry_msgs::PoseStamped const &p2) const  {
    return std::pow((p1.pose.position.x - p2.pose.position.x), 2) +
           std::pow((p1.pose.position.y - p2.pose.position.y), 2);
  };
  nav_msgs::Path polygon_cutter(nav_msgs::Path const &polygon);
  double calculate_polygon_inclination(nav_msgs::Path const &polygon, double tolerance) ;
  void rotate_costmat(const cv::Mat& mat,double angle);
  bool initialize_mats();
  void initialize_neural_mat(cv::Mat const &src, cv::Mat &des) const;
  std::vector<std::vector<SimpleCellIndex>> get_free_cell_in_polygon(cv::Mat const &src, cv::Mat &des,
                                std::vector<CellIndex> &free_space);
  void show_free_cells(const cv::Mat &src, const std::vector<CellIndex> &free_space,
                      const std::vector<std::vector<SimpleCellIndex>> &cells_poses);
  void show_path(const cv::Mat &map, const std::vector<nav_msgs::Path> &paths,
                 const std::string &name);
   void show_result_path(const cv::Mat &map,
                        const nav_msgs::Path &paths,
                        const std::string &name,
                        bool is_island = false);
  void show_astart_path(const cv::Mat &map,
                        const std::vector<cv::Point> &points, int idx);
  void show_miss_path(const cv::Mat &map, const cv::Point &start,
                      const cv::Point &end, const int idx,
                      const nav_msgs::Path &path,
                      bool is_island=false);
  void visualize_backshaped_path(const std::vector<CellIndex> &cells);
  std::vector<nav_msgs::Path> get_coverage_paths(const cv::Point2f& start,const int min_path);
  std::vector<nav_msgs::Path> get_island_paths(const cv::Mat &binmat,
                 std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
                 const std::vector<cv::Point> &boundary,
                 double island_min_area,double island_path_shrink_dis,
                 const cv::Point& start_pt);

  std::vector<nav_msgs::Path> get_island_paths_new(const cv::Mat &binmat,
                 std::shared_ptr<costmap_2d::Costmap2D> const &costmap,
                 const std::vector<cv::Point> &boundary,
                 double island_min_area,double island_path_shrink_dis,
                 const cv::Point& start_pt);

  void adjust_border(nav_msgs::Path const &cover, nav_msgs::Path &border);
  bool save_border(nav_msgs::Path &border);
  cv::Scalar colorFromIndex(int idx, int total);

  std::optional<std::vector<CellIndex>> get_zigzag_indexed_path(const cv::Point2f& start);
  std::optional<std::vector<CellIndex>> get_backshaped_indexed_path(const cv::Point2f& start);
  /*用bezier曲线计算fine_path_*/
  nav_msgs::Path get_bezier_optimized_path(nav_msgs::Path const &path) const;
  void change_weight_tiny(uint32_t first_y);
  std::optional<std::tuple<double, double, uint32_t, uint32_t, CellIndex>>
       get_start_point(const cv::Point2f& start,nav_msgs::Path const &polygon);
  bool is_clockwise(nav_msgs::Path const &path);
  void publish_paths(const CleanRegionInfo &cr_info);
  nav_msgs::Path shrink_polygon(const float shrink_dist,
        const std::vector<cv::Point> &process_ploy);
  bool shrink_Path(const float shrink_dist,nav_msgs::Path &path);
private:
  int model_size_cell_;
  int cell_free_thres_;
  uint8_t lethal_cost_;
  float weight_tiny_;
  double resolution_;
  CleanAreaType clean_area_type_;
  bool debug_=false;
  int obs_inflate_;

private:
  nav_msgs::Path planning_polygon_;
  nav_msgs::Path border_polygon_;
  nav_msgs::Path origin_border_;

  cv::Point2d center_;
  cv::Point2d polygon_center_;

private:
  std::shared_ptr<costmap_2d::Costmap2D> rotated_costmap_ = nullptr;

  cv::Mat rotated_costmat_;
  cv::Mat scale_costmat_;
  cv::Mat neural_costmat_;
  cv::Mat work_binmat_;

  std::vector<std::vector<SimpleCellIndex>> cells_poses_;
  std::vector<CellIndex> free_space_;
  std::vector<nav_msgs::Path> coarse_path_;
  std::vector<nav_msgs::Path> fine_path_;

private:
  AStarFastPlanner astar_fast_planner_;
  double angle_=0.0;
  bool initialized_=false;
};
} // namespace ikitbot::planning
