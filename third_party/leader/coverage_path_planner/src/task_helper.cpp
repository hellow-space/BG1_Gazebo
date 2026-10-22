#include "task_helper.h"

#include <cstdlib>

void TaskHelper::mapToWorld(const int mx, const int my, float &wx, float &wy) {
  // 转回 costmap 左下角索引
  int my_bottom = size_y_ - 1 - my;
  wx = origin_x_ + (mx + 0.5f) * resolution_;
  wy = origin_y_ + (my_bottom + 0.5f) * resolution_;
}

bool TaskHelper::worldToMap(const float wx, const float wy, int &mx, int &my) {
  // 1. 计算原始网格坐标 (基于左下角)
    int mx_bottom = static_cast<int>((wx - origin_x_) / resolution_);
    int my_bottom = static_cast<int>((wy - origin_y_) / resolution_);

    // 记录是否越界
    bool out_of_bounds = (mx_bottom < 0 || mx_bottom >= static_cast<int>(size_x_) || 
                          my_bottom < 0 || my_bottom >= static_cast<int>(size_y_));

    // 2. 越界处理：使用强转后的 int 进行比较
    mx_bottom = std::max(0, std::min(mx_bottom, static_cast<int>(size_x_) - 1));
    my_bottom = std::max(0, std::min(my_bottom, static_cast<int>(size_y_) - 1));

    // 3. 赋值与翻转
    mx = mx_bottom;
    my = static_cast<int>(size_y_) - 1 - my_bottom;

    return !out_of_bounds;
}

std::shared_ptr<costmap_2d::Costmap2D> TaskHelper::initPlanTask(
    const std::string map_name, const nav_msgs::OccupancyGrid &grid,
    cv::Mat &costmat, const double expan_dis, bool debug, std::string &result) {
  work_path_ = std::string(std::getenv("HOME")) + "/maps/" + map_name + "/";
  const std::string region_path = work_path_ + "regions.json";
  const std::string map_path = work_path_ + map_name + ".png";
  const std::string map_yaml_path = work_path_ + map_name + ".yaml";
  const std::string device_path = work_path_ + "device.json";

  // re init
  debug_ = debug;
  map_name_ = map_name;
  world_costmap_ptr_ = nullptr;
  is_docker_exist_ = false;
  regions_.clear();

  // map info
  size_x_ = grid.info.width;
  size_y_ = grid.info.height;
  resolution_ = grid.info.resolution;
  origin_x_ = grid.info.origin.position.x;
  origin_y_ = grid.info.origin.position.y;

  //  加载禁区的相关信息,有可能没有禁区
  if (!loadRegionInfos(region_path)) {
    ROS_WARN("region_path: %s does not exist ... ", region_path.c_str());
  }
  // 加载充电桩
  if (!loadDeviceInfo(device_path)) {
    ROS_WARN(" device_path: %s does not exist ... ", device_path.c_str());
  }
  // 加载电梯
  bool is_elevator_exist = false;
  std::vector<cv::Point2f> elevator;
  is_elevator_exist = getElevatorPolygon(device_path, elevator);

  // grid to costmap
  world_costmap_ptr_ = grid2Costmap(grid);
  // costmap to cv_mat，左上角坐标系
  costmat = costmap2Costmat(world_costmap_ptr_);
  showMap(costmat, "s1_orgin_costmap", kCostMat);

  // 橡皮擦
  eraseRegionOnMat(costmat, true);
  showMap(costmat, "s2_erase_costmap", kCostMat);

  // 电梯禁止规划
  if (is_elevator_exist) {
    markElevatorOnMat(elevator, costmat, true);
  }

  // 禁区应用到地图上
  markRestrictRegionOnMat(expan_dis, costmat, true);

  showMap(costmat, "s3_restrict_region_costmap", kCostMat);
 
  return world_costmap_ptr_;
}
std::shared_ptr<costmap_2d::Costmap2D>
TaskHelper::grid2Costmap(const nav_msgs::OccupancyGrid &grid) {
  auto costmap = std::make_shared<costmap_2d::Costmap2D>(
      grid.info.width, grid.info.height, grid.info.resolution,
      grid.info.origin.position.x, grid.info.origin.position.y);
  // free->0 ,unkonw->255, obs->200~254
  costmap->setDefaultValue(255);

  ROS_INFO(
      "The size of map is [ %d,  %d] with resolution %.4f,origin [%.4f,  %.4f]",
      grid.info.width, grid.info.height, costmap->getResolution(),
      costmap->getOriginX(), costmap->getOriginY());

  costmap->resetMap(0, 0, costmap->getSizeInCellsX(),
                    costmap->getSizeInCellsY());
  for (unsigned y = 0; y < grid.info.height; y++) {
    for (unsigned x = 0; x < grid.info.width; x++) {
      if (grid.data[y * grid.info.width + x] < 0) {
        costmap->setCost(x, y, costmap_2d::NO_INFORMATION);
        continue;
      }

      costmap->setCost(
          x, y,
          static_cast<uint8_t>(grid.data[y * grid.info.width + x] * 254 / 100));
    }
  }

  return costmap;
}
cv::Mat TaskHelper::costmap2Costmat(
    std::shared_ptr<costmap_2d::Costmap2D> const &costmap) {
  // free->0, unkonw->255,obs 1~254 same as costmap
  int x = 0;
  int y = 0;
  auto width = static_cast<int32_t>(costmap->getSizeInCellsX());
  auto height = static_cast<int32_t>(costmap->getSizeInCellsY());
  // 只是改了坐标系，改成左上角为原点
  auto ret_mat = cv::Mat(height, width, CV_8U, cv::Scalar(255));
  for (auto r = 0; r < height; r++) {
    for (auto c = 0; c < width; c++) {
      // costmap's origin is at left bottom ,while opencv's pic's origin is at
      // left-top.
      auto mx = c + x;
      auto my = y + height - r - 1;
      if (mx < 0 || mx >= costmap->getSizeInCellsX() || my < 0 ||
          my >= costmap->getSizeInCellsY()) {
        continue;
      }

      ret_mat.at<uchar>(r, c) = costmap->getCost(static_cast<unsigned>(mx),
                                                 static_cast<unsigned>(my));
    }
  }

  return ret_mat;
}
cv::Mat TaskHelper::costmat2Colormat(const cv::Mat &costmat) {
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

cv::Mat TaskHelper::costmat2Binmat(const cv::Mat &costmat) {
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

bool TaskHelper::loadRegionInfos(std::string region_path) {
  Json::Reader reader;
  Json::Value root;
  ROS_INFO("region_path: %s", region_path.c_str());

  std::ifstream inff(region_path.c_str(), std::ios::in);
  if (!inff.is_open()) {
    ROS_WARN("CANNOT OPEN PATH JSON FILE !!! ");
    return false;
  }

  if (!reader.parse(inff, root)) {
    ROS_INFO("parse region.json fail ");
    return false;
  }

  for (auto r : kRegionMap) {
    Json::Value tmp = root[r.second];
    if (tmp.empty())
      continue;

    for (unsigned int i = 0; i < tmp.size(); i++) {
      Json::Value polygon = tmp[i]["polygon"];
      int id = tmp[i]["id"].asUInt();
      int cnt = polygon.size() / 2;
      std::string reg_type(r.second);
      // ROS_INFO("%s:id=%d,point count=%d", reg_type.c_str(), id, cnt);

      std::vector<cv::Point2f> points;
      if (polygon.size() % 2 != 0) {
        ROS_WARN("RegionstHelper::cvtJson2Local polygon point num is singular");
        continue;
      }

      for (uint j = 0; j < cnt; ++j) {
        cv::Point2f p;
        p.x = polygon[2 * j].asFloat();
        p.y = polygon[2 * j + 1].asFloat();
        points.push_back(p);
      }

      if (points.size()) {
        RegiogInfo info;
        info.emplace(r.first, points);
        regions_.push_back(info);
      }
    }
  }
  ROS_INFO("region size: %d", static_cast<int>(regions_.size()));
  return (regions_.size() > 0);
}
bool TaskHelper::loadDeviceInfo(std::string devices_path) {
  Json::Reader reader;
  Json::Value root;
  ROS_INFO("device_path: %s", devices_path.c_str());

  std::ifstream inff(devices_path.c_str(), std::ios::in);
  if (!inff.is_open()) {
    ROS_WARN("CANNOT OPEN device_path JSON FILE !!! ");
    is_docker_exist_ = false;
    return false;
  }

  std::vector<float> docker;
  if (reader.parse(inff, root)) {
    Json::Value &docker_p = root["charge_station"];
    // ROS_INFO("Read docker info here, amount = %d",
    //          static_cast<int>(docker_p.size()));

    for (unsigned int i = 0; i < docker_p.size() - 1; i++) {
      docker.push_back(docker_p[i].asFloat());
      // ROS_INFO("docker info read[%d] ---- [%.3f]", i, docker_p[i].asFloat());
    }

    if (docker.size() < 4 && docker.size() > 0) {
      // worldToMap(docker[0], docker[1], docker_pose_.x, docker_pose_.y);
      docker_pose_.x = docker[0];
      docker_pose_.y = docker[1];

      ROS_INFO("docker positon [%.3f  %.3f]", docker_pose_.x, docker_pose_.y);
      is_docker_exist_ = true;
      return is_docker_exist_;
    } else {
      ROS_WARN("dock format wrong! ");
    }
  }
  is_docker_exist_ = false;
  return false;
}
bool TaskHelper::eraseRegionOnMat(cv::Mat &map, bool is_costmat) {
  int cnt = 0;
  const cv::Scalar color = is_costmat ? cv::Scalar(0) : cv::Scalar(255);
  for (const auto &reg : regions_) {
    for (const auto &pair : reg) {
      const auto &key = pair.first;
      const auto &value = pair.second;
      int sz = static_cast<int>(value.size());
      if (sz > 2 && key == kEraseType) {
        std::vector<cv::Point> earse_zone;
        for (int i = 0; i < sz; ++i) {
          int mx, my;
          cv::Point zPoint;
          worldToMap(value.at(i).x, value.at(i).y, mx, my);
          zPoint.x = mx;
          zPoint.y = my;
          earse_zone.push_back(zPoint);
        }
        if (earse_zone.size() > 2) {
          cv::fillPoly(map, std::vector<std::vector<cv::Point>>(1, earse_zone),
                       color);
          cnt++;
        }
      }
    }
  }
  ROS_INFO("erase region cnt=%d", cnt);
  return true;
}
bool TaskHelper::markElevatorOnMat(const std::vector<cv::Point2f> &rect,
                                   cv::Mat &map, bool is_costmat) {
  if (rect.size() != 4)
    return false; // 确保是矩形
  const cv::Scalar color = is_costmat ? cv::Scalar(254) : cv::Scalar(0);
  ROS_INFO("mark elevator on map.");
  std::vector<cv::Point> elevator(4);
  for (size_t i = 0; i < 4; ++i) {
    worldToMap(rect[i].x, rect[i].y, elevator[i].x, elevator[i].y);
  }
  
  if( elevator.size()< 2) return false;
  cv::drawContours(map, std::vector<std::vector<cv::Point>>(1, elevator), -1,
                   color, cv::FILLED);
  ROS_INFO("mark elevator on map 2.");
  return true;
}
bool TaskHelper::markCarpetRegionOnMat(const CleanRegionInfo &reg_info,
                                       cv::Mat &map, bool is_costmat) {
  if (reg_info.mode == 0) {
    ROS_INFO("mode is 0, do not need mark the carpet.");
    return false;
  }
  const cv::Scalar color = is_costmat ? cv::Scalar(254) : cv::Scalar(0);
  int cnt = 0;
  for (const auto &reg : regions_) {
    for (const auto &pair : reg) {
      const auto &key = pair.first;
      const auto &value = pair.second;
      int sz = static_cast<int>(value.size());
      if (sz > 2 && key == kCarpetType) {
        ROS_INFO("mark carpet on map sz=%d",sz);
        std::vector<cv::Point> carpet_zone;
        for (int i = 0; i < sz; ++i) {
          int mx, my;
          cv::Point zPoint;
          worldToMap(value.at(i).x, value.at(i).y, mx, my);
          zPoint.x = mx;
          zPoint.y = my;
          carpet_zone.push_back(zPoint);
        }
        // 地毯区与当前清洁有相交时,才填充
        if (carpet_zone.size() > 2) {
          cv::drawContours(map,
                           std::vector<std::vector<cv::Point>>(1, carpet_zone),
                           -1, color, cv::FILLED);
          cnt++;
        }
        
      }
    }
  }
    //  showMap(map, "s3_1_markCarpetRegionOnMat_costmap", kCostMat);
  return true;
}

bool TaskHelper::markRestrictRegionOnMat(const double expan_dis, cv::Mat &map,
                                         bool is_costmat) {
  int int_expan_dis = static_cast<int>(expan_dis * 1000);
  ROS_INFO("Restrict expan_dis=%0.3f,int_expan_dis=%d", expan_dis,
           int_expan_dis);
  const cv::Scalar color = is_costmat ? cv::Scalar(254) : cv::Scalar(0);
  int line_cnt = 0;
  int region_cnt = 0;
  for (const auto &reg : regions_) {
    for (const auto &pair : reg) {
      const auto &key = pair.first;
      const auto &value = pair.second;
      int sz = static_cast<int>(value.size());
      if (key == kRestrictType) {
        // ROS_INFO("mark regions type=%d, points sz=%d", static_cast<int>(key),
        //          sz);
        if (sz == 2) {
          // 禁行区
          cv::Point start_p;
          cv::Point end_p;
          worldToMap(value.at(0).x, value.at(0).y, start_p.x, start_p.y);
          worldToMap(value.at(1).x, value.at(1).y, end_p.x, end_p.y);
          cv::line(map, start_p, end_p, color, 1);
          line_cnt++;
        } else if (sz > 2) {
          // 禁区
          std::vector<cv::Point> region_zone;
          if(abs( int_expan_dis) >0 ){
            clipper::ClipperOffset ploygon;
            clipper::Path i_path;
            clipper::Paths o_paths;
            i_path.clear();
            o_paths.clear();
            for (int i = 0; i < sz; ++i) {
              float wx, wy;
              wx = value.at(i).x;
              wy = value.at(i).y;
              i_path.push_back(
                  clipper::IntPoint((int)(wx * 1000), (int)(wy * 1000)));
            }
            ploygon.AddPath(i_path, clipper::JoinType::jtMiter,
                            clipper::EndType::etClosedPolygon);
            ploygon.Execute(o_paths, int_expan_dis);

            if (o_paths.size()) {
              for (unsigned int r = 0; r < o_paths[0].size(); ++r) {
                int mx1, my1;
                cv::Point zPoint;
                worldToMap(o_paths[0].at(r).X / 1000.0f,
                          o_paths[0].at(r).Y / 1000.0f, mx1, my1);
                zPoint.x = mx1;
                zPoint.y = my1;
                region_zone.push_back(zPoint);
              }
            }
          }else{
            for (int i = 0; i < sz; ++i) {
              float wx, wy;
              cv::Point zPoint;
              worldToMap(value.at(i).x, value.at(i).y, zPoint.x, zPoint.y);
              region_zone.push_back(zPoint);
            }
          }
          
          if (region_zone.size() > 2)
            cv::drawContours(
                map, std::vector<std::vector<cv::Point>>(1, region_zone), -1,
                color, cv::FILLED);
          region_cnt++;
        }
      }
    }
  }
  ROS_INFO("mark restrict region:line=%d ,region=%d", line_cnt, region_cnt);

  return true;
}
bool TaskHelper::getElevatorPolygon(const std::string filename,
                                    std::vector<cv::Point2f> &elevator_rect) {
  Json::Reader reader;
  Json::Value root;

  // 打开 JSON 文件
  std::ifstream inff(filename.c_str(), std::ios::in);
  if (!inff.is_open()) {
    ROS_ERROR("Failed to open file: %s", filename.c_str());
    return false;
  }

  // 解析 JSON 文件
  if (!reader.parse(inff, root)) {
    ROS_ERROR("JSON parse error: %s",
              reader.getFormattedErrorMessages().c_str());
    return false;
  }

  // 检查是否存在 "elevator" 键
  if (!root.isMember("elevator") || !root["elevator"].isArray() ||
      root["elevator"].empty()) {
    ROS_ERROR("No 'elevator' data found in the JSON file.");
    return false;
  }

  // 获取第一个电梯的 polygon 数据
  const Json::Value &elevator = root["elevator"][0];
  if (!elevator.isMember("polygon") || !elevator["polygon"].isArray()) {
    ROS_ERROR("No 'polygon' data found in the 'elevator' object.");
    return false;
  }

  const Json::Value &polygon_data = elevator["polygon"];
  for (int i = 0; i < polygon_data.size(); i += 2) {
    cv::Point2f point;
    point.x = polygon_data[i].asFloat();
    point.y = polygon_data[i + 1].asFloat();
    elevator_rect.push_back(point);
  }
  ROS_INFO("elevator is exist.");
  return true;
}
bool TaskHelper::getDocker(cv::Point2f &docker_pose) {
  if (false == is_docker_exist_){
    docker_pose.x = 0;
    docker_pose.y = 0;
    return false;
  }  
  docker_pose = docker_pose_;
  return true;
}
// 多边形交集
bool TaskHelper::polygonClip(const std::vector<cv::Point> &poly1,
                             const std::vector<cv::Point> &poly2,
                             std::vector<cv::Point> &interPoly) {
  if (poly1.size() < 3 || poly2.size() < 3) {
    return false;
  }

  long x, y;
  // 计算多边形交点
  int count1 = poly1.size();
  int count2 = poly2.size();
  for (int i = 0; i < count1; i++) {
    int poly1_next_idx = (i + 1) % count1;
    for (int j = 0; j < count2; j++) {
      int poly2_next_idx = (j + 1) % count2;
      if (getCrossPoint(poly1[i], poly1[poly1_next_idx], poly2[j],
                        poly2[poly2_next_idx], x, y)) {
        interPoly.push_back(cv::Point(x, y));
      }
    }
  }

  // 计算多边形内部点
  for (int i = 0; i < count1; i++) {
    if (isPointInPolygon(poly2, poly1[i])) {
      interPoly.push_back(poly1[i]);
    }
  }
  for (int i = 0; i < count2; i++) {
    if (isPointInPolygon(poly1, poly2[i])) {
      interPoly.push_back(poly2[i]);
    }
  }

  if (interPoly.size() <= 0)
    return false;

  // 点集排序
  clockwiseSortPoints(interPoly);

  return true;
}
bool TaskHelper::getCrossPoint(const cv::Point &p1, const cv::Point &p2,
                               const cv::Point &q1, const cv::Point &q2,
                               long &x, long &y) {
  if (isRectCross(p1, p2, q1, q2)) {
    if (isLineSegmentCross(p1, p2, q1, q2)) {
      // 求交点
      long tmpLeft, tmpRight;
      tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
      tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) +
                 q1.x * (q2.y - q1.y) * (p2.x - p1.x) -
                 p1.x * (p2.y - p1.y) * (q2.x - q1.x);

      x = (int)((double)tmpRight / (double)tmpLeft);

      tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
      tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) +
                 (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) -
                 q2.y * (q1.x - q2.x) * (p2.y - p1.y);
      y = (int)((double)tmpRight / (double)tmpLeft);
      return true;
    }
  }
  return false;
}

// 顺时针方向排序
void TaskHelper::clockwiseSortPoints(std::vector<cv::Point> &vPoints) {
  // 计算重心
  cv::Point center;
  int count_size = vPoints.size();
  double x = 0, y = 0;
  for (int i = 0; i < count_size; i++) {
    x += vPoints[i].x;
    y += vPoints[i].y;
  }
  center.x = (int)x / count_size;
  center.y = (int)y / count_size;

  // 冒泡排序
  for (int i = 0; i < count_size - 1; i++) {
    for (int j = 0; j < count_size - i - 1; j++) {
      if (pointCompare(vPoints[j], vPoints[j + 1], center)) {
        cv::Point tmp = vPoints[j];
        vPoints[j] = vPoints[j + 1];
        vPoints[j + 1] = tmp;
      }
    }
  }
}
// 点集排序
// 若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool TaskHelper::pointCompare(const cv::Point &a, const cv::Point &b,
                              const cv::Point &center) {
  if (a.x >= 0 && b.x < 0)
    return true;
  if (a.x == 0 && b.x == 0)
    return a.y > b.y;
  // 向量OA和向量OB的叉积
  int det =
      (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
  if (det < 0)
    return true;
  if (det > 0)
    return false;
  // 向量OA和向量OB共线，以距离判断大小
  int d1 =
      (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
  int d2 =
      (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);

  return d1 > d2;
}

// 判断点是否在多边形内部/
bool TaskHelper::isPointInPolygon(const std::vector<cv::Point> &poly,
                                  const cv::Point &pt) {
  int i, j;
  bool c = false;
  int count = poly.size();
  for (i = 0, j = count - 1; i < count; j = i++) {
    if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
         ((poly[j].y <= pt.y) && (pt.y < poly[i].y))) &&
        (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) /
                        (poly[j].y - poly[i].y) +
                    poly[i].x)) {
      c = !c;
    }
  }

  return c;
}
// 排斥实验
bool TaskHelper::isRectCross(const cv::Point &p1, const cv::Point &p2,
                             const cv::Point &q1, const cv::Point &q2) {
  bool ret = std::min(p1.x, p2.x) <= std::max(q1.x, q2.x) &&
             std::min(q1.x, q2.x) <= std::max(p1.x, p2.x) &&
             std::min(p1.y, p2.y) <= std::max(q1.y, q2.y) &&
             std::min(q1.y, q2.y) <= std::max(p1.y, p2.y);

  return ret;
}
// 跨立判断
bool TaskHelper::isLineSegmentCross(const cv::Point &pFirst1,
                                    const cv::Point &pFirst2,
                                    const cv::Point &pSecond1,
                                    const cv::Point &pSecond2) {
  long line1, line2;
  line1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
          pFirst2.x * (pFirst1.y - pSecond1.y) +
          pSecond1.x * (pFirst2.y - pFirst1.y);
  line2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
          pFirst2.x * (pFirst1.y - pSecond2.y) +
          pSecond2.x * (pFirst2.y - pFirst1.y);
  if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
    return false;

  line1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
          pSecond2.x * (pSecond1.y - pFirst1.y) +
          pFirst1.x * (pSecond2.y - pSecond1.y);
  line2 = pSecond1.x * (pFirst2.y - pSecond2.y) +
          pSecond2.x * (pSecond1.y - pFirst2.y) +
          pFirst2.x * (pSecond2.y - pSecond1.y);
  if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
    return false;

  return true;
}
cv::Mat TaskHelper::getWorkCvmat(const cv::Mat &cost_mat,
                                 const std::vector<cv::Point> &ply) {
  // cost_cv_mat to cv_mat:障碍的表示不一样
  const int width = cost_mat.rows;
  const int height = cost_mat.cols;

  // 初始化 opencv 图像(全部设为障碍物 0)
  cv::Mat work_mat(height, width, CV_8UC1, cv::Scalar(0));

  // === 生成 mask （1: 在 polygon 内，0: 外部区域）===
  cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
  const std::vector<std::vector<cv::Point>> contours{ply};
  cv::fillPoly(mask, contours, cv::Scalar(255)); // polygon 内为 255

  // === 填充规划区域 ===
  //   mask 255
  //   mask 0
  for (int r = 0; r < height; r++) {
    for (int c = 0; c < width; c++) {
      // 不在多边形区域 → 障碍物
      if (mask.at<uchar>(r, c) == 0) {
        work_mat.at<uchar>(r, c) = 0;
        continue;
      }

      // 在多边形区域 → 按 costmap 填值
      unsigned char cost = cost_mat.at<uchar>(r, c);
      unsigned char v = (cost == costmap_2d::FREE_SPACE) ? 255 : 0;
      work_mat.at<uchar>(r, c) = v;
    }
  }

  return work_mat;
}
cv::Mat TaskHelper::inflateCostMat(const cv::Mat &cost_mat, int r_pixel) {
  CV_Assert(!cost_mat.empty());
  CV_Assert(cost_mat.type() == CV_8UC1);
  CV_Assert(r_pixel >= 0);
  if (r_pixel == 0) {
    return cost_mat.clone();
  }

  const int rows = cost_mat.rows;
  const int cols = cost_mat.cols;

  uchar inflated_cost = costmap_2d::LETHAL_OBSTACLE;
  // ------------------------------------------------
  // Step 1. 构建障碍 mask（0 = obstacle, 255 = free）
  //        unknown(255) 视为 obstacle
  // ------------------------------------------------
  cv::Mat obstacle_mask(rows, cols, CV_8UC1, cv::Scalar(255));
  for (int y = 0; y < rows; ++y) {
    const uchar *src = cost_mat.ptr<uchar>(y);
    uchar *dst = obstacle_mask.ptr<uchar>(y);

    for (int x = 0; x < cols; ++x) {
      uchar c = src[x];
      // 0           -> free
      // 200~254     -> obstacle
      // 255         -> unknown (treated as obstacle)
      if (c >= 200 || c == 255)
        dst[x] = 0; // obstacle
    }
  }

  // ------------------------------------------------
  // Step 2. 距离变换（到最近 obstacle 的欧式距离）
  // ------------------------------------------------
  cv::Mat dist;
  cv::distanceTransform(obstacle_mask, dist, cv::DIST_L2, 5);

  // ------------------------------------------------
  // Step 3. 生成膨胀后的 costmat
  // ------------------------------------------------
  cv::Mat inflated = cost_mat.clone();
  for (int y = 0; y < rows; ++y) {
    uchar *out = inflated.ptr<uchar>(y);
    const float *dptr = dist.ptr<float>(y);
    for (int x = 0; x < cols; ++x) {
      // 在膨胀半径内
      if (dptr[x] <= static_cast<float>(r_pixel)) {
        // unknown 不被覆盖
        if (out[x] != 255) {
          // 只抬高 cost，不降低已有 cost
          out[x] = std::max(out[x], inflated_cost);
        }
      }
    }
  }

  return inflated;
}
// void TaskHelper::fillEnclosedFreeIslandsOnCostMat2(
//     cv::Mat &costmat, const std::vector<cv::Point> &task_cell_vertices) {
//   CV_Assert(costmat.type() == CV_8UC1);

//   const int rows = costmat.rows;
//   const int cols = costmat.cols;

//   // 1) task_mask: 工作区 polygon
//   cv::Mat task_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
//   cv::drawContours(task_mask,
//                    std::vector<std::vector<cv::Point>>(1,
//                    task_cell_vertices), -1, cv::Scalar(255), cv::FILLED);

//   // 2) task_free: polygon 内的 free + unknown
//   cv::Mat task_free(rows, cols, CV_8UC1, cv::Scalar(255));
//   for (int r = 0; r < rows; ++r) {
//     for (int c = 0; c < cols; ++c) {
//       uchar val = costmat.at<uchar>(r, c);
//       if (task_mask.at<uchar>(r, c) != 0 && (val == 0 || val == 255)) {
//         task_free.at<uchar>(r, c) = 0; // free / unknown 当作可走区域
//       }
//     }
//   }

//   // 3) visited 用于 BFS flood fill
//   cv::Mat visited = cv::Mat::zeros(rows, cols, CV_8UC1);
//   std::queue<cv::Point> q;

//   // 4) flood fill 入队点: polygon 边界上的 free / unknown
//   for (size_t i = 0; i < task_cell_vertices.size(); ++i) {
//     cv::Point p1 = task_cell_vertices[i];
//     cv::Point p2 = task_cell_vertices[(i + 1) % task_cell_vertices.size()];
//     cv::LineIterator it(task_free, p1, p2);
//     for (int k = 0; k < it.count; ++k, ++it) {
//       cv::Point pt = it.pos();
//       if (pt.y >= 0 && pt.y < rows && pt.x >= 0 && pt.x < cols) {
//         if (task_free.at<uchar>(pt.y, pt.x) == 0 &&
//             visited.at<uchar>(pt.y, pt.x) == 0) {
//           visited.at<uchar>(pt.y, pt.x) = 1;
//           q.emplace(pt.x, pt.y);
//         }
//       }
//     }
//   }

//   // 5) BFS flood fill
//   const int dr[4] = {1, -1, 0, 0};
//   const int dc[4] = {0, 0, 1, -1};
//   while (!q.empty()) {
//     cv::Point p = q.front();
//     q.pop();
//     for (int k = 0; k < 4; ++k) {
//       int nr = p.y + dr[k];
//       int nc = p.x + dc[k];
//       if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
//         if (task_free.at<uchar>(nr, nc) == 0 &&
//             visited.at<uchar>(nr, nc) == 0) {
//           visited.at<uchar>(nr, nc) = 1;
//           q.emplace(nc, nr);
//         }
//       }
//     }
//   }

//   // 6) 未访问到的 free / unknown = 真正孤岛 → 填成障碍
//   for (int r = 0; r < rows; ++r) {
//     for (int c = 0; c < cols; ++c) {
//       if (task_mask.at<uchar>(r, c) != 0 && task_free.at<uchar>(r, c) == 0 &&
//           visited.at<uchar>(r, c) == 0) {
//         costmat.at<uchar>(r, c) = 254; // LETHAL_OBSTACLE
//       }
//     }
//   }
// }
void TaskHelper::fillEnclosedFreeIslandsOnCostMat(
    cv::Mat &costmat, const std::vector<cv::Point> &task_cell_vertices) {
  CV_Assert(costmat.type() == CV_8UC1);
  const int rows = costmat.rows;
  const int cols = costmat.cols;

  // 1. 生成工作区掩码
  cv::Mat task_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
  if( task_cell_vertices.size()< 2) return;
  cv::drawContours(task_mask,
                   std::vector<std::vector<cv::Point>>{task_cell_vertices}, -1,
                   cv::Scalar(255), cv::FILLED);

  // 2. 提取工作区内的所有 Free 区域 (白色)
  // 注意：根据你的描述，白色是
  // Free(0或255)，黑色是障碍。这里我们统一提取出白色部分。
  cv::Mat free_binary = cv::Mat::zeros(rows, cols, CV_8UC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      uchar val = costmat.at<uchar>(r, c);
      if (task_mask.at<uchar>(r, c) != 0 && (val == 0 || val == 255)) {
        free_binary.at<uchar>(r, c) = 255;
      }
    }
  }

  // 3. 寻找连通域
  cv::Mat labels, stats, centroids;
  int nLabels = cv::connectedComponentsWithStats(free_binary, labels, stats,
                                                 centroids, 8);

  if (nLabels <= 1)
    return; // 没有 free 区域

  // 4. 找到面积最大的连通域 (忽略背景 label 0)
  int max_area = 0;
  int main_label = -1;
  for (int i = 1; i < nLabels; ++i) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area > max_area) {
      max_area = area;
      main_label = i;
    }
  }

  // 5. 遍历：如果是 Free 区域但不是最大连通域，则视为孤岛并填充
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (task_mask.at<uchar>(r, c) != 0) {
        int label = labels.at<int>(r, c);
        // 如果是白色区域(label > 0) 且 不是主区域
        if (label > 0 && label != main_label) {
          costmat.at<uchar>(r, c) = 254; // 填成 LETHAL_OBSTACLE
        }
      }
    }
  }
}
bool TaskHelper::getTasMatAndBoudPath(
    const cv::Mat &cost_mat, const double task_area_shrink_dist,
    const double boundary_shrink_dist, const int obs_inflate,
    CleanRegionInfo &info, cv::Mat &task_costmat, bool smooth_boud_path) {

  // cost_mat 与costmap的颜色一样
  // task_mat free->255 ,0-->obs

  // ================================
  // Step 1. costmat的处理：膨胀/去孤岛
  // ================================
  task_costmat = inflateCostMat(cost_mat, obs_inflate);
  showCostPlygon(task_costmat, info.task_cell_vertices,
                 "s4_input_inflatcostmat");

  // fillEnclosedFreeIslandsOnCostMat(task_costmat, info.task_cell_vertices);
  // showCostPlygon(task_costmat, info.task_cell_vertices,
  //                "s5_input_encoloseisland_costmat");

  // ================================
  // Step 2. cost → bin (已包含膨胀)
  // ================================
  cv::Mat room_bin = costmat2Binmat(task_costmat);
  if (room_bin.empty()) {
    ROS_ERROR("costmat2Binmat failed");
    return false;
  }

  // ================================
  // Step 3. 初始工作区 mask
  // ================================
  cv::Mat task_mat(room_bin.size(), CV_8UC1, cv::Scalar(0));
  if( info.task_cell_vertices.size()< 2) return false;
  cv::drawContours(
      task_mat, std::vector<std::vector<cv::Point>>(1, info.task_cell_vertices),
      -1, cv::Scalar(255), cv::FILLED);

  // ================================
  // Step 4. 裁剪不可用区域（不膨胀）
  // ================================
  task_mat &= room_bin;

  // ================================
  // Step 5. 第一次提取沿边路径
  // ================================
  info.boundary_path.clear();
  findBoudPath(task_mat, info.boundary_path, smooth_boud_path);

  if (info.boundary_path.empty()) {
    ROS_WARN("empty boundary path after first extraction");
    return false;
  }

  // ================================
  // Step 6. boundary shrink（贴边安全距离）
  // ================================
  if (std::fabs(boundary_shrink_dist) > 1e-3) {
    ROS_INFO("shrink boundary by %.3f", boundary_shrink_dist);
    shrinkPolygon(boundary_shrink_dist, info.boundary_path);
    // 重新生成 task_mat
    task_mat.setTo(0);
    if( info.boundary_path.size()< 2) return false;
    cv::drawContours(task_mat,
                     std::vector<std::vector<cv::Point>>(1, info.boundary_path),
                     -1, cv::Scalar(255), cv::FILLED);

    task_mat &= room_bin;
    // 再次提取真实边界（防止 shrink 后撞障碍）
    info.boundary_path.clear();
    findBoudPath(task_mat, info.boundary_path, smooth_boud_path);
    if (info.boundary_path.empty()) {
      ROS_WARN("empty boundary after boundary shrink");
      return false;
    }
  }

  // showWorkPlygon(task_mat, info.boundary_path, "output_work_ply_boundary");

  // ================================
  // Step 7. task area shrink（给 zigzag / 回字形）
  // ================================
  std::vector<cv::Point> task_vertices = info.boundary_path;
  if (std::fabs(task_area_shrink_dist) > 1e-3) {
    ROS_INFO("shrink task area by %.3f", task_area_shrink_dist);
    shrinkPolygon(task_area_shrink_dist, task_vertices);
    task_mat.setTo(0);
    if( task_vertices.size()< 2) return false;
    cv::drawContours(task_mat,
                     std::vector<std::vector<cv::Point>>(1, task_vertices), -1,
                     cv::Scalar(255), cv::FILLED);

    task_mat &= room_bin;
    // showWorkPlygon(task_mat, task_vertices, "output_work_ply_task");
  }

  // ================================
  // Step 8. 写回 costmat
  // ================================
  fillCostMat(task_costmat, task_vertices, task_costmat);

  return true;
}
void TaskHelper::getTaskCellMinMaxs(
    const std::vector<cv::Point> &task_cell_vertices, int &min_x, int &max_x,
    int &min_y, int &max_y) {
  max_x = 0;
  min_x = 100000;
  max_y = 0;
  min_y = 100000;
  for (size_t idx = 0; idx < task_cell_vertices.size(); ++idx) {
    if (task_cell_vertices[idx].x > max_x)
      max_x = task_cell_vertices[idx].x;
    if (task_cell_vertices[idx].y > max_y)
      max_y = task_cell_vertices[idx].y;
    if (task_cell_vertices[idx].x < min_x)
      min_x = task_cell_vertices[idx].x;
    if (task_cell_vertices[idx].y < min_y)
      min_y = task_cell_vertices[idx].y;
  }

  // std::cout << "\nmax_x, min_x, max_y, min_y[" << max_x << " " << min_x << "
  // "
  //           << max_y << " " << min_y << "]" << std::endl;
}
void TaskHelper::fillBinMat(const cv::Mat &org_map,
                            const std::vector<cv::Point> &vertices,
                            const int erosions_number, cv::Mat &out_map) {
  cv::Mat room_map = org_map.clone();
  if (0 != erosions_number) {
    // 对task cell地图内障碍进行腐蚀,实现障碍物向外扩展
    cv::erode(room_map, room_map, cv::Mat(), cv::Point(-1, -1),
              erosions_number);
  }

  // 依据地图将task cell图填充障碍物的信息
  int max_x, min_x, max_y, min_y;
  getTaskCellMinMaxs(vertices, min_x, max_x, min_y, max_y);
  if (max_x >= room_map.cols)
    max_x = room_map.cols - 1;
  if (max_y >= room_map.rows)
    max_y = room_map.rows - 1;

  for (int v = min_y; v <= max_y; ++v)
    for (int u = min_x; u <= max_x; ++u)
      if (room_map.at<uchar>(v, u) != 255)
        out_map.at<uchar>(v, u) = 0;
}
void TaskHelper::fillCostMat(const cv::Mat &org_costmat,
                             const std::vector<cv::Point> &vertices,
                             cv::Mat &out_costmap) {

  cv::Mat rt_costmat =
      cv::Mat(org_costmat.rows, org_costmat.cols, CV_8U, cv::Scalar(255));
  // 依据地图将task cell图填充障碍物的信息
  int max_x, min_x, max_y, min_y;
  getTaskCellMinMaxs(vertices, min_x, max_x, min_y, max_y);
  if (max_x >= org_costmat.cols)
    max_x = org_costmat.cols - 1;
  if (max_y >= org_costmat.rows)
    max_y = org_costmat.rows - 1;

  for (int v = min_y; v <= max_y; ++v)
    for (int u = min_x; u <= max_x; ++u)
    if (cv::pointPolygonTest(vertices,
                             cv::Point2f(u, v),
                             false) >= 0) 
      rt_costmat.at<uchar>(v, u) = org_costmat.at<uchar>(v, u);
    else
       rt_costmat.at<uchar>(v, u) = 254;

  out_costmap = rt_costmat;
}

void TaskHelper::findBoudPath(const cv::Mat &map, std::vector<cv::Point> &path,
                              bool smooth_boud_path) {
  std::vector<std::vector<cv::Point>> contours_temp;

#if CV_MAJOR_VERSION <= 3
  cv::findContours(map, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,
                   cv::Point());
#else
  cv::findContours(map, contours_temp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
                   cv::Point());
#endif

  if (!contours_temp.empty()) {
    // 获取最大轮廓index
    size_t max_contour_index = 0;
    double max_contour_area = cv::contourArea(contours_temp[0]);
    for (size_t i = 1; i < contours_temp.size(); ++i) {
      double area = cv::contourArea(contours_temp[i]);
      if (area > max_contour_area) {
        max_contour_area = area;
        max_contour_index = i;
      }
    }
    // 获取最大轮廓
    std::vector<cv::Point> max_contour = contours_temp[max_contour_index];
    if (smooth_boud_path) {
      // 如果外层做smooth，此出取精度的边
      path.clear();
      for (auto p : max_contour) {
        path.push_back(p);
      }
    } else {
      // 如果外层不做smooth,通过多边形逼近来优化锯齿
      // 路径下采样/多边形逼近
      std::vector<cv::Point> downsampled_contour;
      cv::approxPolyDP(max_contour, downsampled_contour, 0.5, true);

      // 手动插入第一个点到逼近后的多边形中
      if (!downsampled_contour.empty() &&
          downsampled_contour.front() != downsampled_contour.back()) {
        downsampled_contour.push_back(downsampled_contour.front());
      }
      path.clear();
      for (auto p : downsampled_contour) {
        path.push_back(p);
      }
    }
    // showPath(map,"轮廓",path);
  }
}
bool TaskHelper::shrinkPolygon(const float shrink_dist,
                               std::vector<cv::Point> &process_ploy) {
  clipper::ClipperOffset polygon_offset;
  clipper::Paths o_paths;
  clipper::Path i_path;
  i_path.clear();
  o_paths.clear();

  int int_shrink_dist = static_cast<int>(shrink_dist * 1000);

  // 转成 clipper 坐标（放大1000倍）
  for (int i = 0; i < process_ploy.size(); ++i) {
    float wx, wy;
    mapToWorld(process_ploy[i].x, process_ploy[i].y, wx, wy);
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

  // 1. 简化成无自交多边形
  clipper::Paths simple_paths;
  clipper::SimplifyPolygon(i_path, simple_paths);

  // 2. 收缩每个 simple path，保留所有结果
  for (const auto &path : simple_paths) {
    polygon_offset.Clear();
    polygon_offset.AddPath(path, clipper::jtMiter, clipper::etClosedPolygon);

    clipper::Paths offset_paths;
    polygon_offset.Execute(offset_paths, int_shrink_dist);

    // 合并进最终输出
    o_paths.insert(o_paths.end(), offset_paths.begin(), offset_paths.end());
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

  if (max_path.size() < 3)
    return false;

  // 4. 转换回 map 坐标
  process_ploy.clear();
  for (const auto &pt : max_path) {
    int mx, my;
    worldToMap(pt.X / 1000.0f, pt.Y / 1000.0f, mx, my);
    process_ploy.emplace_back(mx, my);
  }

  return true;
}

void TaskHelper::showMap(const cv::Mat &src, const std::string &tile,
                         uint8_t mat_type) {
  if (false == debug_)
    return;

  cv::Mat tmp;
  switch (mat_type) {
  case kCostMat: {
    tmp = costmat2Colormat(src);
    break;
  }
  default: {
    tmp = src.clone();
    break;
  }
  }
  // cv::namedWindow(tile, cv::WINDOW_NORMAL);
  // cv::resizeWindow(tile, 800, 600);
  // cv::imshow(tile, tmp);
  // cv::waitKey();

  std::string save_path = "/home/pi/tmp/" + tile + ".jpg";
  cv::imwrite(save_path, tmp);
}

void TaskHelper::showWorkPlygon(const cv::Mat src,
                                const std::vector<cv::Point> &vertices,
                                const std::string &tile) {
  if (false == debug_)
    return;
  cv::Mat tmp_map = src.clone();
  cv::cvtColor(tmp_map, tmp_map, cv::COLOR_GRAY2BGR);
  
  if( vertices.size()< 2) return ;
  cv::drawContours(tmp_map, std::vector<std::vector<cv::Point>>(1, vertices),
                   -1, cv::Scalar(0, 0, 255), // RED (BGR)
                   2);

  // cv::namedWindow(tile, cv::WINDOW_NORMAL);
  // cv::resizeWindow(tile, 1000, 1000);
  // cv::imshow(tile, tmp_map);
  // cv::waitKey();

  std::string save_path = "/home/pi/tmp/" + tile + ".jpg";
  cv::imwrite(save_path, tmp_map);
}
void TaskHelper::showCostPlygon(const cv::Mat costmat,
                                const std::vector<cv::Point> &vertices,
                                const std::string &tile) {
  if (false == debug_)
    return;
  cv::Mat tmp = costmat2Colormat(costmat);
  if( vertices.size()< 2) return ;
  cv::drawContours(tmp, std::vector<std::vector<cv::Point>>(1, vertices), -1,
                   cv::Scalar(0, 0, 255), 2);

  std::string save_path = "/home/pi/tmp/" + tile + ".jpg";
  cv::imwrite(save_path, tmp);
}
void TaskHelper::testTran() {
  RegiogInfo info = regions_.at(0);
  const int height = static_cast<int>(world_costmap_ptr_->getSizeInCellsY());

  for (const auto &kv : info) {
    const std::vector<cv::Point2f> &points = kv.second;

    for (const auto &p : points) {
      double wx = p.x;
      double wy = p.y;

      int mx1, my1;
      unsigned int mx2, my2;

      worldToMap(wx, wy, mx1, my1);
      world_costmap_ptr_->worldToMap(wx, wy, mx2, my2);

      ROS_INFO("input(%0.4f,%0.4f),(%d,%d),(%u,%u)", wx, wy, mx1, my1, mx2,
               height - my2 - 1);

      float wx2, wy2;
      double wx3, wy3;
      mapToWorld(mx1, my1, wx2, wy2);
      world_costmap_ptr_->mapToWorld(mx1, height - my1 - 1, wx3, wy3);

      ROS_INFO("input(%d,%d),(%0.4f,%0.4f),(%0.4f,%0.4f)", mx1, my1, wx2, wy2,
               wx3, wy3);
    }
  }
}
bool TaskHelper::assessmentCleanTask(const robot_msg::CoveragePathPlan::Request &req,
                           robot_msg::CoveragePathPlan::Response &resp){
  // 构造结果
  float total_area = 0.0f;
  float total_tm = 0.0f;
  int req_sz = static_cast<int>(req.clean_regions.size());
  int fail_sz = static_cast<int>(resp.fail_region_id.size());
  int resp_sz = static_cast<int>(resp.clean_regions.size());
  for(auto reg : resp.clean_regions){
    total_area += reg.clean_area;
    total_tm += reg.time_require;
  }
  resp.total_area = total_area;
  resp.time_require = total_tm;

  if (fail_sz) {
    ROS_INFO("REQ REGION SZ: %d , FAIL SZ: %d", req_sz, fail_sz);
    resp.result = false;
    resp.failure_reason = "fail for some regions !";
    return false;
  } else if (req_sz == resp_sz) {
    resp.result = true;
    ROS_INFO(" success!, Area: %0.4f,time require:%0.4f", total_area, total_tm);
    return true;
  }
  return false;
}
float TaskHelper::getRegionCleanArea(const cv::Mat &task_costmat,
                           const std::vector<cv::Point> &vPoints){
  if (task_costmat.empty()) {
    return 0.0f;
  }

  int free_cnt = 0;
  for (int r = 0; r < task_costmat.rows; ++r) {
    const uchar* ptr = task_costmat.ptr<uchar>(r);
    for (int c = 0; c < task_costmat.cols; ++c) {
      uchar v = ptr[c];
      if (v == costmap_2d::FREE_SPACE) {
        free_cnt++;
      }
    }
  }

  // 每个像素的实际面积 = resolution * resolution (m^2)
  float area = static_cast<float>(free_cnt) * resolution_ * resolution_;
  return area;
}
float TaskHelper::getRegionCleanTimeRequire(const robot_msg::RegionClean &msg){
  return 0;
}