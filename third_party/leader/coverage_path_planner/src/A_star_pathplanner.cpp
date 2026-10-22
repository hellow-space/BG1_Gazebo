#include "A_star_pathplanner.h"

#include <fstream>

#include "ros/ros.h"

const int dir = 8; // number of possible directions to go at any position
// if dir==4
// static int dx[dir]={1, 0, -1, 0};
// static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

static int expanding_counter = 0;

// // Determine priority (in the priority queue)
// bool operator<(const NodeAstar &a, const NodeAstar &b) {
//   return a.getPriority() > b.getPriority();
// }

AStarPlanner::AStarPlanner() {
  n = 1;
  m = 1;
}

void AStarPlanner::drawRoute(cv::Mat &map, const cv::Point start_point,
                             const std::string &route, double step_length) {
  // follow the route on the map and update the path length
  if (route.length() > 0) {
    int j;
    char c;
    int x1 = start_point.x;
    int y1 = start_point.y;
    int x2, y2;
    for (int i = 0; i < route.length(); i++) {
      // get the next char of the string and make it an integer, which shows the
      // direction
      c = route.at(i);
      j = c - '0';
      x2 = x1 + dx[j] * step_length;
      y2 = y1 + dy[j] * step_length;
      const double progress = 0.2 + 0.6 * (double)i / (double)route.length();
      cv::line(map, cv::Point(x1, y1), cv::Point(x2, y2),
               CV_RGB(0, progress * 255, 0), 1);
      x1 = x2;
      y1 = y2;
    }
  }
}

void AStarPlanner::getRoute(const cv::Point start_point,
                            const std::string &route, double step_length,
                            std::vector<cv::Point> &route_points) {
  // follow the route on the map and update the path length
  if (route.length() > 0) {
    int j;
    char c;
    int x1 = start_point.x;
    int y1 = start_point.y;
    route_points.push_back(cv::Point(x1, y1));
    int x2, y2;
    for (int i = 0; i < route.length(); i++) {
      // get the next char of the string and make it an integer, which shows the
      // direction
      c = route.at(i);
      j = c - '0';
      x2 = x1 + dx[j] * step_length;
      y2 = y1 + dy[j] * step_length;
      route_points.push_back(cv::Point(x2, y2));
      x1 = x2;
      y1 = y2;
    }
  }
}

void AStarPlanner::downsampleMap(const cv::Mat &map, cv::Mat &downsampled_map,
                                 const double downsampling_factor,
                                 const double robot_radius,
                                 const double map_resolution) {
  // erode the map so the planner doesn't go near the walls
  //	--> calculate the number of times for eroding from Robot Radius [m]
  cv::Mat eroded_map;
  int number_of_erosions = (robot_radius / map_resolution);
  cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1, -1), number_of_erosions);
  // downsampling of the map to reduce calculation time
  if (downsampling_factor != 1.)
    cv::resize(eroded_map, downsampled_map, cv::Size(0, 0), downsampling_factor,
               downsampling_factor, cv::INTER_NEAREST); //_LINEAR);
  else
    downsampled_map = eroded_map;
}

// A-star algorithm.
// The route returned is a string of direction digits.
std::string AStarPlanner::pathFind(const int &xStart, const int &yStart,
                                   const int &xFinish, const int &yFinish,
                                   const cv::Mat &map) {
  static std::priority_queue<NodeAstar>
      pq[2];      // list of open (not-yet-tried) nodes
  static int pqi; // pq index
  static NodeAstar *n0;
  static NodeAstar *m0;
  static int i, j, x, y, xdx, ydy;
  static char c;
  pqi = 0;

  cv::Mat map_to_calculate_path(cv::Size(m, n), CV_32S);

  // create map from the given eroded map
  for (int y = 0; y < map.rows; y++) {
    for (int x = 0; x < map.cols; x++) {
      if (map.at<unsigned char>(y, x) == 255) {
        map_to_calculate_path.at<int>(x, y) = 0;
      } else {
        map_to_calculate_path.at<int>(x, y) = 1;
      }
    }
  }

  cv::Mat closed_nodes_map(cv::Size(m, n), CV_32S); // map of already tried
                                                    // nodes
  cv::Mat open_nodes_map(cv::Size(m, n),
                         CV_32S);          // map of open (not-yet-tried) nodes
  cv::Mat dir_map(cv::Size(m, n), CV_32S); // map of directions

  // initialize the node maps
  for (y = 0; y < closed_nodes_map.rows; y++) {
    for (x = 0; x < closed_nodes_map.cols; x++) {
      closed_nodes_map.at<int>(y, x) = 0;
      open_nodes_map.at<int>(y, x) = 0;
    }
  }

  // create the start node and push into list of open nodes
  n0 = new NodeAstar(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);
  open_nodes_map.at<int>(xStart, yStart) =
      n0->getPriority(); // mark it on the open nodes map

  // garbage collection
  delete n0;

  // A* search
  while (!pq[pqi].empty()) {
    // get the current node w/ the highest priority
    // from the list of open nodes
    n0 = new NodeAstar(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                       pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x = n0->getxPos();
    y = n0->getyPos();

    pq[pqi].pop(); // remove the node from the open list
    open_nodes_map.at<int>(x, y) = 0;
    // mark it on the closed nodes map
    closed_nodes_map.at<int>(x, y) = 1;

    // quit searching when the goal state is reached
    // if((*n0).estimate(xFinish, yFinish) == 0)
    if (x == xFinish && y == yFinish) {
      // generate the path from finish to start
      // by following the directions
      std::string path = "";
      while (!(x == xStart && y == yStart)) {
        j = dir_map.at<int>(x, y);
        c = '0' + (j + dir / 2) % dir;
        path = c + path;
        x += dx[j];
        y += dy[j];
      }

      delete n0; // garbage collection
      // empty the leftover nodes
      while (!pq[pqi].empty())
        pq[pqi].pop();
      return path;
    }

    // generate moves (child nodes) in all possible directions
    for (i = 0; i < dir; i++) {
      xdx = x + dx[i];
      ydy = y + dy[i];

      expanding_counter++;

      if (!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 ||
            map_to_calculate_path.at<int>(xdx, ydy) == 1 ||
            closed_nodes_map.at<int>(xdx, ydy) == 1)) {
        // generate a child node
        m0 = new NodeAstar(xdx, ydy, n0->getLevel(), n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if (open_nodes_map.at<int>(xdx, ydy) == 0) {
          open_nodes_map.at<int>(xdx, ydy) = m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent node direction
          dir_map.at<int>(xdx, ydy) = (i + dir / 2) % dir;
        } else if (open_nodes_map.at<int>(xdx, ydy) > m0->getPriority()) {
          // update the priority info
          open_nodes_map.at<int>(xdx, ydy) = m0->getPriority();
          // update the parent direction info
          dir_map.at<int>(xdx, ydy) = (i + dir / 2) % dir;

          // replace the node
          // by emptying one pq to the other one
          // except the node to be replaced will be ignored
          // and the new node will be pushed in instead
          while (!(pq[pqi].top().getxPos() == xdx &&
                   pq[pqi].top().getyPos() == ydy)) {
            pq[1 - pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pq[pqi].pop(); // remove the wanted node

          // empty the larger size pq to the smaller one
          if (pq[pqi].size() > pq[1 - pqi].size())
            pqi = 1 - pqi;
          while (!pq[pqi].empty()) {
            pq[1 - pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pqi = 1 - pqi;
          pq[pqi].push(*m0); // add the better node instead
        }
        delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return ""; // no route found
}

// This is the path planning algorithm for this class. It downsamples the map
// with the given factor (0 < factor < 1) so the map gets reduced and
// calculation time gets better. If it is set to 1 the map will have original
// size, if it is 0 the algorithm won't work, so make sure to not set it to 0.
// The algorithm also needs the Robot radius [m] and the map resolution
// [m²/pixel] to calculate the needed amount of erosions to include the radius
// in the planning.
double AStarPlanner::planPath(const cv::Mat &map, const cv::Point &start_point,
                              const cv::Point &end_point,
                              const double downsampling_factor,
                              const double robot_radius,
                              const double map_resolution,
                              const int end_point_valid_neighborhood_radius,
                              std::vector<cv::Point> *route) {
  // route->clear();
  expanding_counter = 0;
  double step_length = 1. / downsampling_factor;

  // length of the planned path
  double path_length = 0;

  if (start_point.x == end_point.x &&
      start_point.y ==
          end_point.y) // if the start and end-point are the same return 0
  {
    return path_length;
  }

  // check for valid coordinates of start and end points
  if (start_point.x < 0 || start_point.x >= map.cols || start_point.y < 0 ||
      start_point.y >= map.rows || end_point.x < 0 || end_point.x >= map.cols ||
      end_point.y < 0 || end_point.y >= map.rows) {
    return 1e100;
  }

  cv::Mat downsampled_map;
  downsampleMap(map, downsampled_map, downsampling_factor, robot_radius,
                map_resolution);

  // transform the Pixel values to the downsampled ones
  int start_x = downsampling_factor * start_point.x;
  int start_y = downsampling_factor * start_point.y;
  int end_x = downsampling_factor * end_point.x;
  int end_y = downsampling_factor * end_point.y;

  // std::cout << "start_x=" << start_x << ",start_y=" << start_y
  //           << ",end_x=" << end_x << ",end_y=" << end_y << std::endl;
  // set the sizes of the map
  m = downsampled_map.rows; // horizontal size of the map
  n = downsampled_map.cols; // vertical size size of the map

  // get the route
  //	clock_t start = clock();
  route_ = pathFind(start_x, start_y, end_x, end_y, downsampled_map);
  if (route_ == "") {
    if (end_point_valid_neighborhood_radius > 0) {
      for (int r = 1; r <= end_point_valid_neighborhood_radius; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if ((abs(dy) != r && abs(dx) != r) || end_x + dx < 0 ||
                end_x + dx >= n || end_y + dy < 0 || end_y + dy >= m ||
                start_x + dx < 0 || start_x + dx >= n || start_y + dy < 0 ||
                start_y + dy >= m)
              continue;
            route_ = pathFind(start_x, start_y, end_x + dx, end_y + dy,
                              downsampled_map);
            if (route_ != "")
              break;
            route_ = pathFind(start_x + dx, start_y + dy, end_x, end_y,
                              downsampled_map);
            if (route_ != "")
              break;
            route_ = pathFind(start_x + dx, start_y + dy, end_x + dx,
                              end_y + dy, downsampled_map);
            if (route_ != "")
              break;
          }
          if (route_ != "")
            break;
        }
        if (route_ != "")
          break;
      }
    }
    if (route_ == "") {
      // return extremely large distance as path length if the
      // rout could not be generated
      return 1e100;
    }
  }

  // follow the route on the map and update the path length
  if (route_.length() > 0) {
    int j;
    char c;
    int x = start_x;
    int y = start_y;
    const double straight_step = (1. / downsampling_factor);
    const double diagonal_step = (std::sqrt(2.) / downsampling_factor);
    for (int i = 0; i < route_.length(); i++) {
      // get the next char of the string and make it an integer, which shows the
      // direction
      c = route_.at(i);
      j = c - '0';
      x = x + dx[j];
      y = y + dy[j];
      // Update the pathlength with the directions of the path. When the path
      // goes vertical or horizontal add length 1. When it goes diagonal add
      // sqrt(2)
      if (j == 0 || j == 2 || j == 4 || j == 6) {
        path_length += straight_step;
      }
      if (j == 1 || j == 3 || j == 5 || j == 7) {
        path_length += diagonal_step;
      }
    }
  }

  if (route != NULL)
    getRoute(start_point, route_, step_length, *route);

  return path_length;
}

double
AStarPlanner::planPath(const cv::Mat &map, const cv::Mat &downsampled_map,
                       const cv::Point &start_point, const cv::Point &end_point,
                       const double downsampling_factor,
                       const double robot_radius, const double map_resolution,
                       const int end_point_valid_neighborhood_radius,
                       cv::Mat *draw_path_map, std::vector<cv::Point> *route) {
  route_ = "";
  double step_length = 1. / downsampling_factor;
  //	cv::Mat debug = map.clone();
  //	cv::circle(debug, start_point, 2, cv::Scalar(127), CV_FILLED);
  //	cv::circle(debug, end_point, 2, cv::Scalar(127), CV_FILLED);
  //	cv::imshow("debug", debug);
  //	cv::waitKey();
  double pathlength =
      step_length * planPath(downsampled_map, downsampling_factor * start_point,
                             downsampling_factor * end_point, 1., 0.,
                             map_resolution,
                             end_point_valid_neighborhood_radius, route);
  if (pathlength > 1e90) // if no path can be found try with the original map
  {
    pathlength = planPath(
        map, start_point, end_point, 1., 0., map_resolution,
        1. / downsampling_factor * end_point_valid_neighborhood_radius, route);
    step_length = 1.;
  }
  if (pathlength > 1e90)
    std::cout << "## No path found on the originally sized map ###"
              << std::endl;
  else {
    if (draw_path_map != NULL) {
      drawRoute(*draw_path_map, start_point, route_, step_length);
    }
  }

  return pathlength;
}

bool AStarPlanner::smoothPathInt(std::vector<cv::Point> ori_path,
                                 std::vector<cv::Point> &path,
                                 const int count) {
  std::vector<cv::Point2f> old_path;
  std::vector<cv::Point2f> new_path;
  for (auto p : ori_path) {
    float x = p.x * 0.05;
    float y = p.y * 0.05;
    old_path.push_back(cv::Point2f(x, y));
  }

  for (int i = 0; i < count; i++) {
    smoothPath(old_path, new_path);
    old_path = new_path;
  }
  if (new_path.size()) {
    path.clear();
    for (auto p2 : new_path) {
      int x = std::round(p2.x / 0.05);
      int y = std::round(p2.y / 0.05);
      path.push_back(cv::Point(x, y));
    }
  }
}

bool AStarPlanner::smoothPath(std::vector<cv::Point2f> ori_path,
                              std::vector<cv::Point2f> &path) {
  // origin_path密集点等距点，效果会比较好
  cv::Point2f tp = ori_path[0];
  int p_size = ori_path.size();
  path.clear();
  path.push_back(tp);
  if (p_size <= 10) {
    // 点数少，使用平均滤波
    for (int i = 0; i < p_size; i++) {
      tp.x = ori_path[i].x + ori_path[i + 1].x;
      tp.y = ori_path[i].y + ori_path[i + 1].y;
      tp.x = tp.x / 2;
      tp.y = tp.y / 2;

      path.push_back(tp);
    }
  } else {
    // 对于较长的原始路径（超过10个点），使用加权平均滤波来平滑路径
    // step 1: 对于原始路径的前5个点，同样取相邻点的中点作为平滑后路径上的点
    for (int i = 0; i < 5; i++) {
      tp.x = ori_path[i].x + ori_path[i + 1].x;
      tp.y = ori_path[i].y + ori_path[i + 1].y;
      tp.x = tp.x / 2;
      tp.y = tp.y / 2;
      path.push_back(tp);
    }

    // step2:对于路径中间的点,使用一个9点加权平均方法
    path.push_back(tp);
    for (int i = 0; i < (ori_path.size() - 9); i++) {
      tp.x = ori_path[i].x + 9 * ori_path[i + 1].x + 36 * ori_path[i + 2].x +
             84 * ori_path[i + 3].x + 126 * ori_path[i + 4].x +
             126 * ori_path[i + 5].x + 84 * ori_path[i + 6].x +
             36 * ori_path[i + 7].x + 9 * ori_path[i + 8].x + ori_path[i + 9].x;

      tp.y = ori_path[i].y + 9 * ori_path[i + 1].y + 36 * ori_path[i + 2].y +
             84 * ori_path[i + 3].y + 126 * ori_path[i + 4].y +
             126 * ori_path[i + 5].y + 84 * ori_path[i + 6].y +
             36 * ori_path[i + 7].y + 9 * ori_path[i + 8].y + ori_path[i + 9].y;

      tp.x = tp.x / 512;
      tp.y = tp.y / 512;

      path.push_back(tp);
    }
    // step3: 最后4个点
    for (int i = 4; i > 0; i--) {
      tp.x = ori_path[p_size - i - 1].x + ori_path[p_size - i].x;
      tp.y = ori_path[p_size - i - 1].y + ori_path[p_size - i].y;
      tp.x = tp.x / 2;
      tp.y = tp.y / 2;
      path.push_back(tp);
    }
  }
  path.push_back(ori_path.back());

  return true;
}

bool AStarPlanner::smoothPath2(std::vector<cv::Point> ori_path,
                               std::vector<cv::Point> &path,
                               int smoothness_factor) {
  int p_size = ori_path.size();
  path.clear();

  if (p_size == 0)
    return false; // Empty original path

  // Push the first point of the original path
  path.push_back(ori_path[0]);

  if (p_size <= 10) {
    // Simple smoothing for short paths
    for (int i = 0; i < p_size - 1; i++) {
      cv::Point mid_point((ori_path[i].x + ori_path[i + 1].x) / 2,
                          (ori_path[i].y + ori_path[i + 1].y) / 2);
      path.push_back(mid_point);
    }
  } else {
    // Weighted smoothing for longer paths
    for (int i = 0; i < 5; i++) {
      cv::Point mid_point((ori_path[i].x + ori_path[i + 1].x) / 2,
                          (ori_path[i].y + ori_path[i + 1].y) / 2);
      path.push_back(mid_point);
    }

    for (int i = 0; i < (p_size - 9); i++) {
      int x =
          (ori_path[i].x + 9 * ori_path[i + 1].x + 36 * ori_path[i + 2].x +
           84 * ori_path[i + 3].x + 126 * ori_path[i + 4].x +
           126 * ori_path[i + 5].x + 84 * ori_path[i + 6].x +
           36 * ori_path[i + 7].x + 9 * ori_path[i + 8].x + ori_path[i + 9].x) /
          smoothness_factor;
      int y =
          (ori_path[i].y + 9 * ori_path[i + 1].y + 36 * ori_path[i + 2].y +
           84 * ori_path[i + 3].y + 126 * ori_path[i + 4].y +
           126 * ori_path[i + 5].y + 84 * ori_path[i + 6].y +
           36 * ori_path[i + 7].y + 9 * ori_path[i + 8].y + ori_path[i + 9].y) /
          smoothness_factor;
      cv::Point weighted_point(x, y);
      path.push_back(weighted_point);
    }

    for (int i = 4; i > 0; i--) {
      cv::Point mid_point(
          (ori_path[p_size - i - 1].x + ori_path[p_size - i].x) / 2,
          (ori_path[p_size - i - 1].y + ori_path[p_size - i].y) / 2);
      path.push_back(mid_point);
    }
  }

  // Push the last point of the original path
  path.push_back(ori_path.back());

  return true;
}

bool AStarPlanner::smoothPathByDir(std::vector<cv::Point2f> ori_path,
                                   std::vector<cv::Point2f> &path,
                                   double angle_threshold) {
  // Ensure path contains at least two points
  if (ori_path.size() < 2) {
    return false;
  }

  // Add the first point to the smoothed path
  path.push_back(ori_path.front());

  // Iterate through the original path
  for (size_t i = 1; i < ori_path.size() - 1; ++i) {
    // Calculate the direction vector from the previous point to the current
    // point
    cv::Point dir = ori_path[i] - path.back();

    // Calculate the direction vector between the current point and the previous
    // point
    cv::Point prev_dir = path.back() - path[path.size() - 2];

    // Calculate the angle between the current direction and the previous
    // direction
    double angle =
        cv::fastAtan2(dir.y, dir.x) - cv::fastAtan2(prev_dir.y, prev_dir.x);

    // Ensure angle is within range [-180, 180]
    while (angle < -180.0)
      angle += 360.0;
    while (angle > 180.0)
      angle -= 360.0;

    // Check if the absolute angle change is larger than the threshold
    if (std::abs(angle) > angle_threshold) {
      // If the angle change is larger than the threshold, add the current point
      // to the smoothed path
      path.push_back(ori_path[i]);
    }
  }

  // Add the last point to the smoothed path
  path.push_back(ori_path.back());

  return true;
}
void AStarPlanner::savePoints(const std::string &file_path,
                              const std::vector<cv::Point2f> &points) {
  // 删除已存在的文件
  std::remove(file_path.c_str());

  // 创建一个输出文件流
  std::ofstream file(file_path);

  // 检查文件是否成功打开
  if (!file.is_open()) {
    std::cerr << "Error: Unable to open file for writing: " << file_path
              << std::endl;
    return;
  }

  // 将点的信息写入文件
  for (const auto &point : points) {
    file << point.x << " " << point.y << std::endl;
  }

  // 关闭文件流
  file.close();

  std::cout << "Points saved to file: " << file_path << std::endl;
}

void AStarPlanner::smoothInteriorAngle(const std::vector<cv::Point2f> &ori_path,
                                       std::vector<cv::Point2f> &path,
                                       double angle_threshold,
                                       double shrink_distance) {
  if (ori_path.size() < 3) {
    std::cerr << "Error: Input path should contain at least three points."
              << std::endl;
    return;
  }

  path.clear();

  for (size_t i = 0; i < ori_path.size(); ++i) {
    cv::Point2f prev_point =
        ori_path[(i + ori_path.size() - 1) % ori_path.size()];
    cv::Point2f current_point = ori_path[i];
    cv::Point2f next_point = ori_path[(i + 1) % ori_path.size()];

    // 计算两向量的模长
    double norm_vec1 = cv::norm(current_point - prev_point);
    double norm_vec2 = cv::norm(next_point - current_point);

    // 检查模长是否为零，如果为零，则跳过该点
    if (norm_vec1 == 0 || norm_vec2 == 0) {
      continue;
    }

    // 计算两向量的叉积
    double cross_product =
        (current_point.x - prev_point.x) * (next_point.y - current_point.y) -
        (next_point.x - current_point.x) * (current_point.y - prev_point.y);

    // 计算夹角的弧度值
    double angle_radians = std::acos(cross_product / (norm_vec1 * norm_vec2));

    // 将弧度转换为度
    double angle_degrees = angle_radians * 180 / CV_PI;

    // 检查角度是否小于阈值
    if (angle_degrees < angle_threshold) {
      // 计算缩进的点
      cv::Point2f shrink_vec =
          prev_point - current_point + next_point - current_point;
      cv::Point2f shrink_point =
          current_point -
          shrink_vec * (shrink_distance / (norm_vec1 + norm_vec2));

      // 添加到输出路径
      path.push_back(shrink_point);
    } else {
      // 直接添加当前点到输出路径
      path.push_back(current_point);
    }
  }
}
bool AStarPlanner::insertPointForPath(const std::vector<cv::Point2f> &ori_path,
                                      std::vector<cv::Point2f> &path,
                                      double dist) {
  std::vector<cv::Point2f> path_out;

  if (ori_path.size() < 2) {
    path = ori_path;
    return false;
  }

  size_t sz = ori_path.size() - 1;
  cv::Point2f point = ori_path[0];
  for (size_t i = 0; i < sz; i++) {
    double theta = std::atan2(ori_path[i + 1].y - ori_path[i].y,
                              ori_path[i + 1].x - ori_path[i].x);
    double length = cv::norm(ori_path[i + 1] - ori_path[i]);
    size_t insert_size = static_cast<size_t>(length / dist + 0.5);

    double dx = dist * cos(theta);
    double dy = dist * sin(theta);
    point = ori_path[i];

    path_out.push_back(point);

    for (size_t j = 1; j < insert_size; j++) {
      point.x = point.x + dx;
      point.y = point.y + dy;
      path_out.push_back(point);
    }
  }
  point = ori_path.back();
  path_out.push_back(point);
  path.clear();
  sz = path_out.size();
  path.resize(sz);
  for (size_t i = 0; i < sz; i++) {
    path[i] = path_out[i];
  }
  return true;
}

void AStarPlanner::sheftPathCorner(const std::vector<cv::Point2f> &ori_path,
                                   std::vector<cv::Point2f> &path,
                                   double shrink_distance) {
  PathPoint tp;

  std::vector<PathPoint> path_smooth;
  std::vector<PathPoint> corner_pts;
  std::vector<int> corner_idx;
  std::vector<double> corner_idx_r;
  std::vector<PathPoint> out_path;

  // 转成带角度及长度的点云
  int path_size = ori_path.size();
  if (path_size < 3) {
    path.clear();
    path = ori_path;
    std::cout << "sheftPathCorner:input path is too small!" << std::endl;
  }

  // 第一个点
  PathPoint pt1;
  pt1.x = ori_path[0].x;
  pt1.y = ori_path[0].y;
  pt1.theta = 0.0f;
  pt1.s = 0.0f;
  path_smooth.push_back(pt1);
  for (int i = 1; i < path_size; i++) {
    double dx = ori_path[i].x - ori_path[i - 1].x;
    double dy = ori_path[i].y - ori_path[i - 1].y;
    if (fabs(dx) < 0.005 && fabs(dy) < 0.005) {
      continue;
    }
    double theta = atan2(dy, dx);
    double len = sqrt(dx * dx + dy * dy);
    double base_s = path_smooth.back().s;

    PathPoint p;
    p.x = ori_path[i].x;
    p.y = ori_path[i].y;
    p.theta = theta;
    p.s = base_s + len;

    path_smooth.push_back(p);
  }

  // 更新第一个点的角度
  if (path_smooth.size() < 3) {
    path.clear();
    path = ori_path;
    std::cout << "sheftPathCorner: path is too small!" << std::endl;
  }

  path_smooth[0].theta = path_smooth[1].theta;

  EdgePathCorner edgePathCorner(2000000, 1.0, 0.01, 80);
  // // 插值放到外部
  // path_smooth = edgePathCorner.interplotPath(path_smooth);
  edgePathCorner.getEdgePathCorner(path_smooth, corner_pts, corner_idx,
                                   corner_idx_r);

  out_path =
      edgePathCorner.sheftCornerPose(path_smooth, corner_idx, corner_idx_r);

  path.clear();
  for (const auto &point : out_path) {
    cv::Point2f p;
    p.x = point.x;
    p.y = point.y;
    path.push_back(p);
  }
}

std::vector<PathPoint>
EdgePathCorner::sheftCornerPose(std::vector<PathPoint> &path,
                                std::vector<int> &corner_idx,
                                std::vector<double> &corner_r) {
  std::vector<PathPoint> path_offset;

  int corner_sz = corner_idx.size();
  if (corner_sz < 3) {
    printf("not find path corner! \n");
    return path_offset;
  }
  std::vector<std::vector<int>> corner_cells_idx;
  std::vector<int> cell;
  cell.push_back(corner_idx[0]);
  int cell_kd_size = 40;
  int sheft_length = 30;

  for (int i = 1; i < corner_sz; ++i) {
    if (abs(cell.back() - corner_idx[i]) > cell_kd_size ||
        (corner_r[i] * corner_r[i - 1] < 0)) {
      if (cell.size() > 6) {
        corner_cells_idx.push_back(cell);
      }
      cell.clear();
    }
    cell.push_back(corner_idx[i]);
  }

  // signedArea = calculateSignedArea(path);
  double circle_dir = (signedArea > 0) ? 1 : -1;

  corner_cells_idx.push_back(cell);
  ;

  int path_sz = path.size();
  std::vector<CornerIdx> cell_pts;
  for (size_t i = 0; i < corner_cells_idx.size(); ++i) {
    int front = corner_cells_idx[i].front();
    int back = corner_cells_idx[i].back();
    int mid = std::floor(0.5 * (front + back));
    int sheft_front = front - sheft_length;
    int sheft_back = back + sheft_length;

    if (sheft_front <= 0) {
      sheft_front = 1;
    }

    if (sheft_back >= path_sz) {
      sheft_back = path_sz - 1;
    }

    if (i < corner_cells_idx.size() - 1) {
      if (sheft_back > 0.5 * (back + corner_cells_idx[i + 1].front())) {
        sheft_back = std::floor(0.5 * (back + corner_cells_idx[i + 1].front()));
      }
    }

    if (i > 0) {
      if (sheft_front < 0.5 * (front + corner_cells_idx[i - 1].back())) {
        sheft_front =
            std::floor(0.5 * (front + corner_cells_idx[i - 1].back()));
      }
    }

    CornerIdx cell_ps = {static_cast<unsigned short>(front),
                         static_cast<unsigned short>(mid),
                         static_cast<unsigned short>(back),
                         static_cast<unsigned short>(sheft_front),
                         static_cast<unsigned short>(sheft_back)};
    cell_pts.push_back(cell_ps);
  }

#if 0
        std::ofstream outfile4("/mnt/hgfs/repository/corner_pts_sheft.txt");
        for(int i = 0; i< cell_pts.size(); i++) {
            outfile4 << path[cell_pts[i].sheft_front].x << " " << path[cell_pts[i].sheft_front].y << std::endl;
            outfile4 << path[cell_pts[i].front].x << " " << path[cell_pts[i].front].y << std::endl;
            outfile4 << path[cell_pts[i].mid].x << " " << path[cell_pts[i].mid].y << std::endl;
            outfile4 << path[cell_pts[i].back].x << " " << path[cell_pts[i].back].y << std::endl;
            outfile4 << path[cell_pts[i].sheft_back].x << " " << path[cell_pts[i].sheft_back].y << std::endl;
        }
        outfile4.close();
#endif

  path_offset.clear();
  double offset_dist = 0.05;
  for (size_t i = 0; i < cell_pts.size(); ++i) {
    if (i == 0) {
      path_offset.insert(path_offset.end(), path.begin(),
                         path.begin() + cell_pts[i].sheft_front);

      std::vector<PathPoint> path_front = offsetCornerPath(
          path, cell_pts[i].sheft_front, cell_pts[i].front, cell_pts[i].mid,
          cell_pts[i].back, cell_pts[i].sheft_back, circle_dir, offset_dist);
      path_offset.insert(path_offset.end(), path_front.begin(),
                         path_front.end());
    } else if (i == cell_pts.size() - 1) {
      if (cell_pts[i].sheft_front - cell_pts[i - 1].sheft_back < 20) {
        std::vector<PathPoint> path_back =
            offsetCornerPath(path, cell_pts[i], circle_dir, offset_dist);
        std::vector<PathPoint> line =
            interplotLine(path_offset.back(), path_back[0], 0.01);
        path_offset.insert(path_offset.end(), line.begin(), line.end());

        // printf("\n\n\n p_size = %d, cell_pts(%d, %d, %d, %d, %d) \n\n\n",
        // path.size(),cell_pts[i].sheft_back, cell_pts[i].back,
        // cell_pts[i].mid, cell_pts[i].front, cell_pts[i].sheft_front);

        path_offset.insert(path_offset.end(), path_back.begin(),
                           path_back.end());

        path_offset.insert(path_offset.end(),
                           path.begin() + cell_pts[i].sheft_back, path.end());

      } else {
        path_offset.insert(path_offset.end(),
                           path.begin() + cell_pts[i - 1].sheft_back,
                           path.begin() + cell_pts[i].sheft_front);

        std::vector<PathPoint> path_back =
            offsetCornerPath(path, cell_pts[i], circle_dir, offset_dist);
        path_offset.insert(path_offset.end(), path_back.begin(),
                           path_back.end());

        path_offset.insert(path_offset.end(),
                           path.begin() + cell_pts[i].sheft_back, path.end());
      }
    } else {
      if (cell_pts[i].sheft_front - cell_pts[i - 1].sheft_back < 20) {
        std::vector<PathPoint> path_back =
            offsetCornerPath(path, cell_pts[i], circle_dir, offset_dist);
        std::vector<PathPoint> line =
            interplotLine(path_offset.back(), path_back[0], 0.01);
        path_offset.insert(path_offset.end(), line.begin(), line.end());
        path_offset.insert(path_offset.end(), path_back.begin(),
                           path_back.end());
      } else {
        path_offset.insert(path_offset.end(),
                           path.begin() + cell_pts[i - 1].sheft_back,
                           path.begin() + cell_pts[i].sheft_front);
        std::vector<PathPoint> path_back =
            offsetCornerPath(path, cell_pts[i], circle_dir, offset_dist);
        path_offset.insert(path_offset.end(), path_back.begin(),
                           path_back.end());
      }
    }
  }

  return path_offset;
}
///////////////////////////////////////////////////
std::vector<PathPoint>
EdgePathCorner::interplotPath(std::vector<PathPoint> &path_smooth) {
  int path_smooth_size = path_smooth.size();
  std::vector<PathPoint> path;
  path.push_back(path_smooth[0]);
  path[0].s = 0;
  for (int i = 1; i < path_smooth_size; i++) {
    double dx = path_smooth[i].x - path_smooth[i - 1].x;
    double dy = path_smooth[i].y - path_smooth[i - 1].y;
    if (fabs(dx) < 0.005 && fabs(dy) < 0.005) {
      continue;
    }
    double theta = atan2(dy, dx);
    double len = sqrt(dx * dx + dy * dy);
    double nx = dx / len;
    double ny = dy / len;
    double base_s = path.back().s;
    for (double s = res; s < len; s += res) {
      double x = path_smooth[i - 1].x + nx * s;
      double y = path_smooth[i - 1].y + ny * s;
      path.push_back({x, y, theta, base_s + s});
    }
    path.push_back({path_smooth[i].x, path_smooth[i].y, theta, base_s + len});
  }
  path[0].theta = path[1].theta;
  return path;
}

double EdgePathCorner::calculateSignedArea(std::vector<PathPoint> &polygon) {
  double area = 0;
  int n = polygon.size();
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    area += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
  }
  return area / 2.0f;
}
std::vector<PathPoint>
EdgePathCorner::offsetCornerPath(std::vector<PathPoint> &path,
                                 CornerIdx corner_idx, double circle_dir,
                                 double offset_dist) {
  return offsetCornerPath(path, corner_idx.sheft_front, corner_idx.front,
                          corner_idx.mid, corner_idx.back,
                          corner_idx.sheft_back, circle_dir, offset_dist);
}

std::vector<PathPoint>
EdgePathCorner::offsetCornerPath(std::vector<PathPoint> &path, int sheft_front,
                                 int front, int mid, int back, int sheft_back,
                                 double circle_dir, double offset_dist) {
  std::vector<PathPoint> offset_path;

  // Calculate normals
  double n0[2] = {-circle_dir * sin(path[sheft_front].theta),
                  circle_dir * cos(path[sheft_front].theta)};
  double n1[2] = {-circle_dir * sin(path[front].theta),
                  circle_dir * cos(path[front].theta)};
  double n2[2] = {-circle_dir * sin(path[mid].theta),
                  circle_dir * cos(path[mid].theta)};
  double n3[2] = {-circle_dir * sin(path[back].theta),
                  circle_dir * cos(path[back].theta)};
  double n4[2] = {-circle_dir * sin(path[sheft_back].theta),
                  circle_dir * cos(path[sheft_back].theta)};

  // Create concatenated path
  std::vector<PathPoint> p;
  p.push_back(path[sheft_front]);
  p.push_back(path[front]);
  p.push_back(path[mid]);
  p.push_back(path[back]);
  p.push_back(path[sheft_back]);

  // Offset each point
  p[0].x += n0[0] * offset_dist;
  p[0].y += n0[1] * offset_dist;

  p[1].x += n1[0] * offset_dist;
  p[1].y += n1[1] * offset_dist;

  p[2].x += n2[0] * 2 * offset_dist;
  p[2].y += n2[1] * 2 * offset_dist;

  p[3].x += n3[0] * offset_dist;
  p[3].y += n3[1] * offset_dist;

  p[4].x += n4[0] * offset_dist;
  p[4].y += n4[1] * offset_dist;

  // Generate offset path
  offset_path.push_back(p[0]);
  for (size_t i = 1; i < p.size(); ++i) {
    double dx = p[i].x - p[i - 1].x;
    double dy = p[i].y - p[i - 1].y;
    double ds = sqrt(dx * dx + dy * dy);
    dx /= ds;
    dy /= ds;
    PathPoint pt = p[i - 1];
    for (double s = 0.01; s < ds; s += 0.01) {
      pt.x = p[i - 1].x + dx * s;
      pt.y = p[i - 1].y + dy * s;
      offset_path.push_back(pt);
    }
  }
  offset_path.push_back(p[4]);

  return offset_path;
}

std::vector<PathPoint>
EdgePathCorner::interplotLine(PathPoint &sp, PathPoint &ep, double res) {
  std::vector<PathPoint> interpolated_points;
  double dx = ep.x - sp.x;
  double dy = ep.y - sp.y;
  double s = std::sqrt(dx * dx + dy * dy);
  dx = dx / s;
  dy = dy / s;

  PathPoint cell = ep;
  cell.theta = std::atan2(dy, dx);
  cell.s = s;

  for (double ts = res; ts <= s; ts += res) {
    cell.x = sp.x + ts * dx;
    cell.y = sp.y + ts * dy;
    cell.s = sp.s + ts;
    interpolated_points.push_back(cell);
  }
  interpolated_points.push_back(cell);

  return interpolated_points;
}

void EdgePathCorner::getEdgePathCorner(std::vector<PathPoint> &path_smooth,
                                       std::vector<PathPoint> &corner_pts,
                                       std::vector<int> &corner_idx,
                                       std::vector<double> &corner_idx_r) {
  int path_smooth_size = path_smooth.size();

  std::vector<double> r_kap(path_smooth_size);
  int half_delta_i = delta_i / 2;
  for (int i = 0; i < path_smooth_size; i++) {
    double th, s;
    if (i <= delta_i) {
      th = path_smooth[delta_i].theta - path_smooth[0].theta;
      s = path_smooth[delta_i].s - path_smooth[0].s;
    } else if (i >= (path_smooth_size - delta_i)) {
      th = path_smooth[path_smooth_size - 1].theta -
           path_smooth[path_smooth_size - 1 - delta_i].theta;
      s = path_smooth[path_smooth_size - 1].s -
          path_smooth[path_smooth_size - 1 - delta_i].s;
    } else {
      th = path_smooth[i + half_delta_i].theta -
           path_smooth[i - half_delta_i].theta;
      s = path_smooth[i + half_delta_i].s - path_smooth[i - half_delta_i].s;
    }
    if (th < -M_PI) {
      th += 2 * M_PI;
    } else if (th > M_PI) {
      th -= 2 * M_PI;
    }
    r_kap[i] = (fabs(th) > 0.00001) ? s / th : const_max_r;
  }

  for (int i = 2; i < path_smooth_size - 2; i++) {
    if (fabs(r_kap[i]) < conner_r) {
      corner_idx.push_back(i);
      corner_pts.push_back({path_smooth[i].x, path_smooth[i].y});
      corner_idx_r.push_back(r_kap[i]);
    }
  }
}
