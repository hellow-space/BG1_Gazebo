/**
 * @file        task_helper.h
 * @brief       地图信息/清洁区/禁区数据文件管理
 * @details
 * @author      colin
 * @date        2023-12-14
 * @version     V1.0.0
 * @copyright   Copyright (c) 2021-2023  ikitbot
 */
#pragma once

#include "clean_region.h"
#include "curve.h"
#include "json.h"
#include "yaml_helper.h"
#include <robot_msg/CoveragePathPlan.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <cmath>
#include <fstream>
#include <iomanip> // 包含 std::fixed 和 std::setprecision
#include <iostream>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <sstream> // 包含 std::ostringstream
#include <string>
#include <vector>

static const uint8_t kRestrictType = 1;
static const uint8_t kSlopeType = 2;
static const uint8_t kLvzType = 3;
static const uint8_t kHighLightType = 4;
static const uint8_t kCarpetType = 5;
static const uint8_t kEraseType = 6;

static const uint8_t kColorMat = 1;
static const uint8_t kBinMat = 2;
static const uint8_t kCostMat = 3;

static std::unordered_map<int, std::string> kRegionMap{
    {kRestrictType, "restrict"}, {kSlopeType, "slope"},
    {kLvzType, "lvz"},           {kHighLightType, "highlight"},
    {kCarpetType, "carpet"},     {kEraseType, "erase"}};
using RegiogInfo = std::map<uint8_t, std::vector<cv::Point2f>>;

/**
 * @class TaskHelper
 * @brief task管理类
 */
class TaskHelper {
public:
  TaskHelper() { ros::NodeHandle nh_("~"); }
  ~TaskHelper() {}

  std::shared_ptr<costmap_2d::Costmap2D>
  initPlanTask(const std::string map_name, const nav_msgs::OccupancyGrid &grid,
               cv::Mat &costmat, const double expan_dis,bool debug,std::string &result);

  void mapToWorld(const int mx, const int my, float &wx, float &wy);
  bool worldToMap(const float wx, const float wy, int &mx, int &my);

  bool getDocker(cv::Point2f &docker_pose);
  bool markCarpetRegionOnMat(const CleanRegionInfo &reg_info, cv::Mat &map,
                             bool is_costmat);
  cv::Mat getWorkCvmat(const cv::Mat &cost_mat,
                         const std::vector<cv::Point> &ply);

  bool getTasMatAndBoudPath(const cv::Mat &cos_tmat,
                             const double task_area_shrink_dist,
                             const double boundary_shrink_dist,
                             const int obs_inflate, CleanRegionInfo &info,
                             cv::Mat &task_costmat, bool smooth_boud_path = true);
  

  void showMap(const cv::Mat &src, const std::string &tile,uint8_t mat_type);
  
  void testTran();
  // void pushBackRegionMsg(robot_msg::RegionClean msg) { clean_regions_.push_back(msg); }
  bool assessmentCleanTask(const robot_msg::CoveragePathPlan::Request &req,
                           robot_msg::CoveragePathPlan::Response &resp);
  float getRegionCleanArea(const cv::Mat &task_costmat,
                           const std::vector<cv::Point> &vPoints);
  float getRegionCleanTimeRequire(const robot_msg::RegionClean &msg);

private:
  std::shared_ptr<costmap_2d::Costmap2D>
  grid2Costmap(const nav_msgs::OccupancyGrid &grid);
  cv::Mat
  costmap2Costmat(const std::shared_ptr<costmap_2d::Costmap2D> &cost_map);
  cv::Mat costmat2Colormat(const cv::Mat &costmat);
  cv::Mat costmat2Binmat(const cv::Mat &costmat);
  cv::Mat inflateCostMat(const cv::Mat& cost_mat,int r_pixel);
  void fillEnclosedFreeIslandsOnCostMat(cv::Mat &costmat,
                  const std::vector<cv::Point> &task_cell_vertices);

private:
  bool loadRegionInfos(std::string region_path);
  bool loadDeviceInfo(std::string devices_path);
  bool getElevatorPolygon(const std::string filename,
                          std::vector<cv::Point2f> &elevator_rect);
  bool eraseRegionOnMat(cv::Mat &map, bool is_costmat);
  bool markElevatorOnMat(const std::vector<cv::Point2f> &rect, cv::Mat &map,
                         bool is_costmat);
  bool markRestrictRegionOnMat(const double expan_dis, cv::Mat &map,
                               bool is_costmat);
  void fillBinMat(const cv::Mat &org_map, const std::vector<cv::Point> &vertices,
               const int erosions_number, cv::Mat &out_map);
  void fillCostMat(const cv::Mat &org_costmat, const std::vector<cv::Point> &vertices,
                cv::Mat &out_costmap);

  void findBoudPath(const cv::Mat &map, std::vector<cv::Point> &path,
                    bool smooth_boud_path = false);
  void getTaskCellMinMaxs(const std::vector<cv::Point> &task_cell_vertices,
                          int &min_x, int &max_x, int &min_y, int &max_y);
  void showWorkPlygon(const cv::Mat src,const std::vector<cv::Point> &vertices,const std::string& tile);
  void showCostPlygon(const cv::Mat costmat,const std::vector<cv::Point> &vertices,const std::string& tile);
private:
  bool polygonClip(const std::vector<cv::Point> &poly1,
                   const std::vector<cv::Point> &poly2,
                   std::vector<cv::Point> &interPoly);
  bool getCrossPoint(const cv::Point &p1, const cv::Point &p2,
                     const cv::Point &q1, const cv::Point &q2, long &x,
                     long &y);

  bool isRectCross(const cv::Point &p1, const cv::Point &p2,
                   const cv::Point &q1, const cv::Point &q2);
  bool isLineSegmentCross(const cv::Point &pFirst1, const cv::Point &pFirst2,
                          const cv::Point &pSecond1, const cv::Point &pSecond2);
  bool isPointInPolygon(const std::vector<cv::Point> &poly,
                        const cv::Point &pt);
  void clockwiseSortPoints(std::vector<cv::Point> &vPoints);
  bool pointCompare(const cv::Point &a, const cv::Point &b,
                    const cv::Point &center);
  bool shrinkPolygon(const float shrink_dist,
                     std::vector<cv::Point> &process_ploy);

private:
  unsigned int size_x_; ///< @brief 地图的宽
  unsigned int size_y_; ///< @brief 地图的高
  float resolution_;    ///< @brief 地图的分辨率
  float origin_x_;      ///< @brief 地图的原点x坐标
  float origin_y_;      ///< @brief 地图的原点y坐标

  bool debug_=false;

  std::string work_path_;
  std::string map_name_;

  std::vector<RegiogInfo> regions_;
  // std::vector<robot_msg::RegionClean> clean_regions_;

  cv::Point2d docker_pose_;
  bool is_docker_exist_ = false;

  std::shared_ptr<costmap_2d::Costmap2D> world_costmap_ptr_ = nullptr;
  cv::Mat cv_costmat_;
};
