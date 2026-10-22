#include "A_star_pathplanner_fast.h"
#include "ros/ros.h"
#include <fstream>

// 强制清理优先级队列的辅助工具
template <class T>
void clearPriorityQueue(T& q) {
    T empty;
    std::swap(q, empty);
}

void AStarFastPlanner::prepareMap(const cv::Mat &map, double downsample_factor,
                                  int obs_inflate, double map_resolution) {
    downsample_factor_ = downsample_factor;
    obs_inflate_ = obs_inflate;
    map_resolution_ = map_resolution;

    // 1) 腐蚀处理
    cv::Mat eroded;
    if (obs_inflate_ > 0) {
        cv::erode(map, eroded, cv::Mat(), cv::Point(-1, -1), obs_inflate_);
        cv::imwrite("/home/pi/tmp/s7_astart_map.jpg", eroded);
    } else {
        eroded = map.clone();
    }
    
    // 2) 下采样
    if (std::abs(downsample_factor - 1.0) > 1e-4) {
        cv::resize(eroded, ds_map_, cv::Size(0, 0), downsample_factor,
                   downsample_factor, cv::INTER_NEAREST);
    } else {
        ds_map_ = eroded.clone();
    }

    // 更新当前地图尺寸
    rows_ = ds_map_.rows;
    cols_ = ds_map_.cols;

    // 3) 障碍物矩阵转换 - 强制重新创建以确保内存连续且尺寸精确
    obstacle_ = cv::Mat(rows_, cols_, CV_8U);
    for (int y = 0; y < rows_; y++) {
        const uint8_t* map_ptr = ds_map_.ptr<uint8_t>(y);
        uint8_t* obs_ptr = obstacle_.ptr<uint8_t>(y);
        for (int x = 0; x < cols_; x++) {
            obs_ptr[x] = (map_ptr[x] >= 250 ? 0 : 1);
        }
    }

    // 4) 关键：重置 A* 辅助矩阵
    // 使用 create 会在尺寸变化时重新分配内存，防止越界
    closed_.create(rows_, cols_, CV_16U);
    closed_.setTo(cv::Scalar(0));
    
    open_.create(rows_, cols_, CV_32S);
    open_.setTo(cv::Scalar(0));
    
    parent_dir_.create(rows_, cols_, CV_8S);
    parent_dir_.setTo(cv::Scalar(-1));

    // 5) 彻底清空优先级队列，防止残留
    clearPriorityQueue(pq_[0]);
    clearPriorityQueue(pq_[1]);
}
bool AStarFastPlanner::isLineFree(int x1, int y1, int x2, int y2) {
    cv::LineIterator it(obstacle_, cv::Point(x1, y1), cv::Point(x2, y2), 8);
    for (int i = 0; i < it.count; i++, ++it) {
        if (obstacle_.at<uint8_t>(it.pos()) == 1) return false;
    }
    return true;
}
std::string AStarFastPlanner::pathFind(int xs, int ys, int xe, int ye) {
    // 基础边界防御
    if (xs < 0 || xs >= cols_ || ys < 0 || ys >= rows_ || 
        xe < 0 || xe >= cols_ || ye < 0 || ye >= rows_) return "";
    
    // 检查起点终点是否本身就是障碍物
    if (obstacle_.at<uint8_t>(ys, xs) == 1 || obstacle_.at<uint8_t>(ye, xe) == 1) return "";

    // 每次搜索前必须清零辅助矩阵（不重新分配内存，只清空内容）
    closed_.setTo(0);
    open_.setTo(0);
    parent_dir_.setTo(-1);

    clearPriorityQueue(pq_[0]);
    clearPriorityQueue(pq_[1]);
    int pqi = 0;

    NodeAstar start(xs, ys, 0, 0);
    start.updatePriority(xe, ye);
    pq_[pqi].push(start);
    open_.at<int>(ys, xs) = start.getPriority();

    static const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    // 设定最大搜索次数防止死循环（内存溢出后的异常情况）
    int max_iterations = rows_ * cols_;
    int iterations = 0;

    while (!pq_[pqi].empty() && iterations++ < max_iterations) {
        NodeAstar cur = pq_[pqi].top();
        pq_[pqi].pop();

        int x = cur.getxPos();
        int y = cur.getyPos();

        // 已经检查过该点
        if (closed_.at<uint16_t>(y, x) == 1) continue;
        
        open_.at<int>(y, x) = 0;
        closed_.at<uint16_t>(y, x) = 1;

        // 到达终点
        if (x == xe && y == ye) {
            std::string path = "";
            int curr_x = x, curr_y = y;
            // 回溯保护，防止死循环
            int path_limit = rows_ * cols_;
            while (!(curr_x == xs && curr_y == ys) && path_limit-- > 0) {
                int8_t d = parent_dir_.at<int8_t>(curr_y, curr_x);
                if (d == -1) break; // 异常回溯
                int c = (d + 4) % 8;
                path = char('0' + c) + path;
                curr_x += dx[d];
                curr_y += dy[d];
            }
            return path;
        }

        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx < 0 || nx >= cols_ || ny < 0 || ny >= rows_) continue;
            if (obstacle_.at<uint8_t>(ny, nx) == 1 || closed_.at<uint16_t>(ny, nx) == 1) continue;

            NodeAstar n2(nx, ny, cur.getLevel(), cur.getPriority());
            n2.nextLevel(i);
            n2.updatePriority(xe, ye);

            int existing_priority = open_.at<int>(ny, nx);
            if (existing_priority == 0 || existing_priority > n2.getPriority()) {
                open_.at<int>(ny, nx) = n2.getPriority();
                parent_dir_.at<int8_t>(ny, nx) = (i + 4) % 8;
                pq_[pqi].push(n2);
            }
        }
    }
    return "";
}

std::string AStarFastPlanner::getRouteWithNeighborhood(int xs, int ys, int xe, int ye, int radius) {
    // 1. 如果起点终点重合，直接返回空路径
    if (xs == xe && ys == ye) return "";

    // 2. 预检：如果起点或终点在障碍物中，先找最近的 Free 点
    int best_sx = xs, best_sy = ys;
    int best_ex = xe, best_ey = ye;

    auto find_nearest_free = [&](int start_x, int start_y, int& out_x, int& out_y) -> bool {
        if (obstacle_.at<uint8_t>(start_y, start_x) == 0) return true;
        int r_limit = std::max(radius, 10);
        for (int r = 1; r <= r_limit; r++) {
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    if (std::abs(dx) != r && std::abs(dy) != r) continue;
                    int nx = start_x + dx, ny = start_y + dy;
                    if (nx >= 0 && nx < cols_ && ny >= 0 && ny < rows_ && obstacle_.at<uint8_t>(ny, nx) == 0) {
                        out_x = nx; out_y = ny; return true;
                    }
                }
            }
        }
        return false;
    };

    if (!find_nearest_free(xs, ys, best_sx, best_sy)) return "";
    if (!find_nearest_free(xe, ye, best_ex, best_ey)) return "";

    // 3. 直线连通检测（极速剪枝）
    if (isLineFree(best_sx, best_sy, best_ex, best_ey)) {
        // 构建直线链码 (简单实现：这里可以返回特殊标记，或直接让 pathFind 跑一个极短的 A*)
        // 为简化逻辑，我们让 pathFind 处理这种简单情况
    }

    // 4. 执行唯一一次 A* 搜索
    return pathFind(best_sx, best_sy, best_ex, best_ey);
}


double AStarFastPlanner::planPathFast(const cv::Point &start, const cv::Point &goal, 
                                      int end_radius, std::vector<cv::Point> &out_path) {
    // 坐标映射
    int xs = std::round(start.x * downsample_factor_);
    int ys = std::round(start.y * downsample_factor_);
    int xe = std::round(goal.x * downsample_factor_);
    int ye = std::round(goal.y * downsample_factor_);

    if (xs < 0 || xs >= cols_ || ys < 0 || ys >= rows_ || xe < 0 || xe >= cols_ || ye < 0 || ye >= rows_) return 1e100;

    // 1. 直线连通检测 (物理单位/原始分辨率)
    // 如果在下采样地图上直线可达，直接生成路径点，跳过所有 A* 逻辑
    if (isLineFree(xs, ys, xe, ye)) {
        out_path.clear();
        out_path.push_back(start);
        out_path.push_back(goal);
        return cv::norm(start - goal);
    }

    // 2. 邻域搜索 + 唯一 A* 规划
    std::string route = getRouteWithNeighborhood(xs, ys, xe, ye, end_radius);
    if (route.empty()) return 1e100;

    // 3. 路径还原
    out_path.clear();
    const double step_length = 1.0 / downsample_factor_;
    static const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    double x_curr = static_cast<double>(start.x);
    double y_curr = static_cast<double>(start.y);
    out_path.emplace_back(std::round(x_curr), std::round(y_curr));

    for (char c : route) {
        int j = c - '0';
        x_curr += dx[j] * step_length;
        y_curr += dy[j] * step_length;
        out_path.emplace_back(std::round(x_curr), std::round(y_curr));
    }
    return static_cast<double>(out_path.size());
}

double AStarFastPlanner::planPathFast(const cv::Point &start, const cv::Point &goal) {
    
    // 1. 坐标映射与边界检查
    int xs = start.x * downsample_factor_;
    int ys = start.y * downsample_factor_;
    int xe = goal.x * downsample_factor_;
    int ye = goal.y * downsample_factor_;

    // 边界检查：如果起点或终点在地图外，返回无穷大距离
    if (xs < 0 || xs >= cols_ || ys < 0 || ys >= rows_) return 1e100;
    if (xe < 0 || xe >= cols_ || ye < 0 || ye >= rows_) return 1e100;

    // 2. 获取路径链码
    // 注意：原函数有一个 end_radius 参数，如果是点对点规划，通常设为 0
    // 如果类成员中有默认半径，请替换这里的 0
    int end_radius = 0; 
    std::string route = getRouteWithNeighborhood(xs, ys, xe, ye, end_radius);

    if (route.empty()) {
        // 无法找到路径
        return 1e100;
    }

    // 3. 计算路径长度
    // 预计算直线步长和对角线步长
    const double step_len = 1.0 / downsample_factor_;
    const double diag_len = step_len * 1.41421356237; // sqrt(2) * step_len

    double total_length = 0.0;

    // 遍历链码计算总长
    // 假设 dx/dy 定义如下 (与您提供的原版保持一致):
    // indices:      0, 1, 2,  3,  4,  5,  6,  7
    // dx:           1, 1, 0, -1, -1, -1,  0,  1
    // dy:           0, 1, 1,  1,  0, -1, -1, -1
    // 观察规律：
    // 偶数索引 (0,2,4,6) -> (1,0), (0,1)... 是直线移动 (Cardinal)
    // 奇数索引 (1,3,5,7) -> (1,1), (-1,1)... 是对角线移动 (Diagonal)
    
    for (char c : route) {
        // 将字符 '0'-'7' 转换为整数索引
        int direction_index = c - '0';

        // 使用位运算判断奇偶性 (direction_index & 1) 
        // 奇数为对角线，偶数为直线
        if (direction_index & 1) {
            total_length += diag_len;
        } else {
            total_length += step_len;
        }
    }

    // 可选：如果需要考虑从路径末端到精确 goal 的剩余微小距离，可以在这里加上
    // 但通常 A* 在栅格化后，算到这一步已经足够精确了

    return total_length;
}
bool AStarFastPlanner::isFree(const cv::Point2f &p) {
    int xs = std::round(p.x * downsample_factor_);
    int ys = std::round(p.y * downsample_factor_);
    if (xs < 0 || xs >= cols_ || ys < 0 || ys >= rows_) return false;
    return (obstacle_.at<uint8_t>(ys, xs) == 0);
}

cv::Point AStarFastPlanner::ensureFreePoint(const cv::Point &p) {
    if (isFree(cv::Point2f(p.x, p.y))) return p;
    for (int r = 1; r <= 10; ++r) { // 扩大搜索半径
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                cv::Point np(p.x + dx, p.y + dy);
                if (isFree(cv::Point2f(np.x, np.y))) return np;
            }
        }
    }
    return p; 
}

bool AStarFastPlanner::smoothPath(std::vector<cv::Point> ori_path,
                                  std::vector<cv::Point> &path,
                                  int smoothness_factor) {
    if (ori_path.size() < 3) {
        path = ori_path;
        return true;
    }
    path.clear();
    path.push_back(ori_path.front());

    for (size_t i = 1; i < ori_path.size() - 1; ++i) {
        // 简单的 3 点平滑，降低“切角”力度
        int sx = (ori_path[i-1].x + 2 * ori_path[i].x + ori_path[i+1].x) / 4;
        int sy = (ori_path[i-1].y + 2 * ori_path[i].y + ori_path[i+1].y) / 4;
        cv::Point smoothed_p(sx, sy);

        // --- 关键：穿墙保护 ---
        if (isFree(cv::Point2f(smoothed_p.x, smoothed_p.y))) {
            path.push_back(smoothed_p);
        } else {
            path.push_back(ori_path[i]); // 穿墙则使用原始点
        }
    }
    path.push_back(ori_path.back());
    return true;
}
bool AStarFastPlanner::smoothPath2(std::vector<cv::Point> ori_path,
                               std::vector<cv::Point> &path,
                               int smoothness_factor) {
  int p_size = ori_path.size();
  path.clear();

  if (p_size == 0) return false;  // Empty original path

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

void AStarFastPlanner::smoothPath3(const std::vector<cv::Point>& raw_path, std::vector<cv::Point>& smoothed_path) {
    if (raw_path.size() <= 2) {
        smoothed_path = raw_path;
        return;
    }

    int n = raw_path.size();
    smoothed_path = raw_path; // 初始拷贝一份

    // 平滑参数
    const int window_size = 5; // 窗口越大越圆滑，建议 3-7
    const double weight_data = 0.5;   // 保持原位置的权重
    const double weight_smooth = 0.5; // 向相邻点靠拢的权重

    // 进行多次迭代平滑 (通常 2-5 次)
    for (int iter = 0; iter < 5; ++iter) {
        for (int i = 1; i < n - 1; ++i) {
            double old_x = smoothed_path[i].x;
            double old_y = smoothed_path[i].y;
            
            // 核心公式：当前点 = 当前点 + 保持权重 * (原始点 - 当前点) + 平滑权重 * (前一点 + 后一点 - 2 * 当前点)
            double new_x = old_x + weight_data * (raw_path[i].x - old_x) + 
                           weight_smooth * (smoothed_path[i-1].x + smoothed_path[i+1].x - 2.0 * old_x);
            double new_y = old_y + weight_data * (raw_path[i].y - old_y) + 
                           weight_smooth * (smoothed_path[i-1].y + smoothed_path[i+1].y - 2.0 * old_y);

            // 碰撞检测：平滑后的点如果进障碍了，就保留原样
            cv::Point2f pt(new_x,new_y);
            if (isFree(pt)) {
                smoothed_path[i].x = std::round(new_x);
                smoothed_path[i].y = std::round(new_y);
            }
        }
    }
}

