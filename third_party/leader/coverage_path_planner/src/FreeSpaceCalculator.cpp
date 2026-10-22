//
// Created by tony on 2022/10/25.
//

#include "FreeSpaceCalculator.h"

using namespace ikitbot::planning;

auto FreeSpaceCalculator::find_bound_of_free_space(
    std::vector<CellIndex> const &free_space, cv::Mat const &neural_mat)
    -> std::tuple<int32_t, int32_t, int32_t, int32_t> {
  if (free_space.empty()) {
    return std::make_tuple(0, 0, neural_mat.cols, neural_mat.rows);
  }

  auto min_max_x = std::minmax_element(
      free_space.begin(), free_space.end(),
      [&](CellIndex const &cell_a, CellIndex const &cell_b) {
        return cell_a.col < cell_b.col;
      });
  auto min_max_y = std::minmax_element(
      free_space.begin(), free_space.end(),
      [&](CellIndex const &cell_a, CellIndex const &cell_b) {
        return cell_a.row < cell_b.row;
      });
  return std::make_tuple(min_max_x.first->col, min_max_y.first->row,
                         min_max_x.second->col - min_max_x.first->col + 1,
                         min_max_y.second->row - min_max_y.first->row + 1);
}


auto FreeSpaceCalculator::get_free_cell_in_polygon2(
    cv::Mat const &src, cv::Mat &des, std::vector<CellIndex> &free_space,
    nav_msgs::Path const &planning_polygon,
    std::shared_ptr<costmap_2d::Costmap2D> const &rotated_costmap,
    bool border_contained) -> int32_t {

  if (!des.empty() || des.data != nullptr) {
    des.release();
  }

  des = cv::Mat(
      src.rows / model_size_cell_ + (src.rows % model_size_cell_ == 0 ? 0 : 1),
      src.cols / model_size_cell_ + (src.cols % model_size_cell_ == 0 ? 0 : 1),
      src.type());

  std::vector<CellIndex> polygon;
  int a, b;
  bool duplicated;

  for (auto const &pose : planning_polygon.poses) {
    duplicated = false;
    rotated_costmap->worldToMapEnforceBounds(pose.pose.position.x,
                                             pose.pose.position.y, a, b);
    for (auto const &point : polygon) {
      if (point.row == (src.rows - b - 1) / model_size_cell_ &&
          point.col == a / model_size_cell_) {
        duplicated = true;
      }
    }

    if (!duplicated) {
      CellIndex tmp;
      tmp.row = (src.rows - b - 1) / model_size_cell_;
      tmp.col = a / model_size_cell_;
      polygon.emplace_back(tmp);
    }
  }

  if (polygon.size() < 3) {
    ROS_ERROR("Got a polygon zone with less than 3 points. Give up coverage "
              "path planning.");
  }

  free_space.clear();
  cells_poses_.clear();

  for (int r = 0; r < des.rows; r++) {
    std::vector<SimpleCellIndex> row_vec;
    for (int c = 0; c < des.cols; c++) {
      CellIndex ci{r, c};
      row_vec.emplace_back(SimpleCellIndex{-1, -1});

      auto in_polygon_state = in_polygon_cell(ci, polygon);
      if (in_polygon_state == InPolygonState::out_of_polygon) {
        des.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
        continue;
      }

      // 1. 收集所有合法的子像素（这里要把所有在 src 范围内的点都列为候选）
      std::vector<SimpleCellIndex> all_points;
      for (int i = 0; i < model_size_cell_; ++i) {
        for (int j = 0; j < model_size_cell_; ++j) {
          int curr_r = r * model_size_cell_ + i;
          int curr_c = c * model_size_cell_ + j;
          if (curr_r >= 0 && curr_r < src.rows && curr_c >= 0 &&
              curr_c < src.cols) {
            all_points.emplace_back(SimpleCellIndex{i, j});
          }
        }
      }

      if (all_points.empty())
        continue;

      // 2. 核心优化筛选逻辑
      auto best_elem = std::min_element(
          all_points.begin(), all_points.end(),
          [&](SimpleCellIndex const &p1, SimpleCellIndex const &p2) {
            int center = model_size_cell_ / 2;
            uchar cost1 = src.at<uchar>(r * model_size_cell_ + p1.row,
                                        c * model_size_cell_ + p1.col);
            uchar cost2 = src.at<uchar>(r * model_size_cell_ + p2.row,
                                        c * model_size_cell_ + p2.col);

            // 优先级 1: Cost 越小越安全 (远离障碍)
            if (cost1 != cost2) {
              return cost1 < cost2;
            }

            // 优先级 2: 距离中心越近越好 (路径直)
            int dist1 = std::abs(p1.row - center) + std::abs(p1.col - center);
            int dist2 = std::abs(p2.row - center) + std::abs(p2.col - center);
            return dist1 < dist2;
          });

      // 3. 最终判定：只有选出的最安全点其 Cost 小于阈值，才认为该格子是 Free
      uchar best_cost = src.at<uchar>(r * model_size_cell_ + best_elem->row,
                                      c * model_size_cell_ + best_elem->col);

      if (best_cost < cell_free_thres_) {
        row_vec.back().row = best_elem->row;
        row_vec.back().col = best_elem->col;
        des.at<uchar>(r, c) = costmap_2d::FREE_SPACE;
        free_space.emplace_back(ci);
      } else {
        des.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
      }
    }
    cells_poses_.emplace_back(row_vec);
  }
  return src.rows - des.rows * model_size_cell_;
}

auto FreeSpaceCalculator::get_free_cell_in_polygon(
    cv::Mat const &src, cv::Mat &des, std::vector<CellIndex> &free_space,
    nav_msgs::Path const &planning_polygon,
    std::shared_ptr<costmap_2d::Costmap2D> const &rotated_costmap,
    bool border_contained ) -> int32_t {

    if (!des.empty()) des.release();

    // 1. 初始化 des 尺寸与原图对齐的网格（保持兼容性）
    int des_rows = src.rows / model_size_cell_ + (src.rows % model_size_cell_ == 0 ? 0 : 1);
    int des_cols = src.cols / model_size_cell_ + (src.cols % model_size_cell_ == 0 ? 0 : 1);
    des = cv::Mat(des_rows, des_cols, src.type(), cv::Scalar(costmap_2d::LETHAL_OBSTACLE));
    
    free_space.clear();
    cells_poses_.assign(des.rows, std::vector<SimpleCellIndex>(des.cols, {-1, -1}));

    // 2. 转换多边形到像素坐标并计算包围盒
    int min_px = src.cols, max_px = 0, min_py = src.rows, max_py = 0;
    std::vector<cv::Point> pixel_points;
    for (auto const &pose : planning_polygon.poses) {
        int mx, my;
        rotated_costmap->worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, mx, my);
        int cur_x = mx;
        int cur_y = src.rows - my - 1; 
        pixel_points.emplace_back(cur_x, cur_y);
        min_px = std::min(min_px, cur_x); max_px = std::max(max_px, cur_x);
        min_py = std::min(min_py, cur_y); max_py = std::max(max_py, cur_y);
    }

    // 3. 渲染多边形掩码 (严禁路径出界的核心)
    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
    if (!pixel_points.empty()) {
        std::vector<std::vector<cv::Point>> fill_polylines = {pixel_points};
        cv::fillPoly(mask, fill_polylines, cv::Scalar(255));
    }

    // 4. 核心逻辑：计算起始偏移量 (Offset) 解决留白问题
    // 我们不再死板地从图像 0,0 开始切格子，而是从多边形的左边缘开始
    // 但我们依然把结果存入对应的 des(r, c) 中
    
    // 计算多边形区域内可以放下多少个点
    int num_points_x = std::max(1, (max_px - min_px) / model_size_cell_ + 1);
    int num_points_y = std::max(1, (max_py - min_py) / model_size_cell_ + 1);

    for (int i = 0; i < num_points_y; ++i) {
        for (int j = 0; j < num_points_x; ++j) {
            
            // 计算当前“路径点”在原图上的精确像素坐标
            // 通过从 min_px/py 开始，保证了“贴边”，不留白
            int curr_px = min_px + j * model_size_cell_;
            int curr_py = min_py + i * model_size_cell_;

            // --- 检查1：是否在多边形内 ---
            if (curr_px >= src.cols || curr_py >= src.rows || mask.at<uchar>(curr_py, curr_px) == 0) {
                continue;
            }

            // --- 检查2：碰撞检测 ---
            bool collision = false;
            int radius = model_size_cell_ / 2;
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int py = std::clamp(curr_py + dy, 0, src.rows - 1);
                    int px = std::clamp(curr_px + dx, 0, src.cols - 1);
                    if (src.at<uchar>(py, px) >= 200) {
                        collision = true;
                        break;
                    }
                }
                if (collision) break;
            }

            if (!collision) {
                // --- 坐标映射回 des 矩阵 ---
                // 我们计算这个像素坐标属于 des 矩阵的哪个格子
                int r = curr_py / model_size_cell_;
                int c = curr_px / model_size_cell_;

                if (r >= 0 && r < des.rows && c >= 0 && c < des.cols) {
                    des.at<uchar>(r, c) = costmap_2d::FREE_SPACE;
                    free_space.emplace_back(CellIndex{r, c});

                    // 记录像素偏置：这是关键！
                    // target_pose 是像素坐标相对于该网格(r, c)左上角的偏移
                    cells_poses_[r][c].row = curr_py - (r * model_size_cell_);
                    cells_poses_[r][c].col = curr_px - (c * model_size_cell_);
                }
            }
        }
    }

    return 0; 
}
auto FreeSpaceCalculator::get_free_cell_in_polygon_final(
    cv::Mat const &src, cv::Mat &des, std::vector<CellIndex> &free_space,
    nav_msgs::Path const &planning_polygon,
    std::shared_ptr<costmap_2d::Costmap2D> const &rotated_costmap) -> int32_t {

    if (!des.empty()) des.release();

    // 1. 获取边界 (保持不变)
    float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    std::vector<cv::Point> pixel_points;
    for (auto const &pose : planning_polygon.poses) {
        unsigned int mx, my;
        if (rotated_costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
            float cur_x = static_cast<float>(mx);
            float cur_y = static_cast<float>(src.rows - static_cast<int>(my) - 1);
            pixel_points.emplace_back(cv::Point(std::round(cur_x), std::round(cur_y)));
            min_x = std::min(min_x, cur_x); max_x = std::max(max_x, cur_x);
            min_y = std::min(min_y, cur_y); max_y = std::max(max_y, cur_y);
        }
    }

    int des_rows = (src.rows + model_size_cell_ - 1) / model_size_cell_;
    int des_cols = (src.cols + model_size_cell_ - 1) / model_size_cell_;
    des = cv::Mat(des_rows, des_cols, CV_8UC1, cv::Scalar(costmap_2d::LETHAL_OBSTACLE));
    cells_poses_.assign(des_rows, std::vector<SimpleCellIndex>(des_cols, {-1, -1}));
    
    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{pixel_points}, cv::Scalar(255));

    // 2. 【中间对齐】逻辑
    float center_x = (min_x + max_x) / 2.0f;
    float center_y = (min_y + max_y) / 2.0f;

    // 计算在此区域内能放下多少个完整的 cell
    int num_x = std::floor((max_x - min_x) / model_size_cell_);
    int num_y = std::floor((max_y - min_y) / model_size_cell_);

    // 计算第一个 cell 的起始中心位置，使其相对于多边形中心对称
    float start_px = center_x - (num_x - 1) * model_size_cell_ / 2.0f;
    float start_py = center_y - (num_y - 1) * model_size_cell_ / 2.0f;

    free_space.clear();

    // 3. 遍历每个格子
    for (int i = 0; i < num_y; ++i) {
        for (int j = 0; j < num_x; ++j) {
            // 理论上的格子中心
            int ideal_px = std::round(start_px + j * model_size_cell_);
            int ideal_py = std::round(start_py + i * model_size_cell_);

            // 4. 【偏心采样逻辑】
            int final_px = -1, final_py = -1;
            bool found_free = false;
            bool has_obstacle = false;

            // 在该 Cell 的范围内 (model_size_cell_ x model_size_cell_) 寻找 free 像素
            int r = model_size_cell_ / 2;
            for (int dy = -r; dy <= r && !has_obstacle; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    int tx = ideal_px + dx;
                    int ty = ideal_py + dy;

                    if (tx < 0 || tx >= src.cols || ty < 0 || ty >= src.rows) continue;
                    
                    uchar val = src.at<uchar>(ty, tx);
                    
                    // 规则 2：绝对不能有障碍物 (200-254)
                    if (val >= 200 && val < 255) {
                        has_obstacle = true;
                        break;
                    }

                    // 规则 3：寻找 free 像素 (0)，优先取中心
                    if (val == 0 && mask.at<uchar>(ty, tx) == 255) {
                        if (!found_free) {
                            final_px = tx;
                            final_py = ty;
                            found_free = true;
                        } else {
                            // 如果已经找到了 free 点，但当前点更接近中心，则更新
                            if (std::abs(dx) + std::abs(dy) < std::abs(final_px - ideal_px) + std::abs(final_py - ideal_py)) {
                                final_px = tx;
                                final_py = ty;
                            }
                        }
                    }
                }
            }

            // 5. 只有找到 free 像素且无障碍时才生成
            if (found_free && !has_obstacle) {
                int des_c = ideal_px / model_size_cell_;
                int des_r = ideal_py / model_size_cell_;

                if (des_r >= 0 && des_r < des.rows && des_c >= 0 && des_c < des.cols) {
                    des.at<uchar>(des_r, des_c) = costmap_2d::FREE_SPACE;
                    // 存储实际找到的那个“偏心”后的 Free 像素位置
                    cells_poses_[des_r][des_c].row = final_py - (des_r * model_size_cell_);
                    cells_poses_[des_r][des_c].col = final_px - (des_c * model_size_cell_);
                    free_space.emplace_back(CellIndex{des_r, des_c});
                }
            }
        }
    }
    return 0;
}
auto FreeSpaceCalculator::in_polygon_map(
    CellIndex const &cell, nav_msgs::Path const &planning_polygon,
    std::shared_ptr<costmap_2d::Costmap2D> const &rotated_costmap) const
    -> InPolygonState {
  auto mx_of_cell =
      static_cast<int32_t>(cell.col * model_size_cell_ + model_size_cell_ / 2);
  auto my_of_cell = static_cast<int32_t>(
      rotated_costmap->getSizeInCellsY() -
      (cell.row * model_size_cell_ + model_size_cell_ / 2) - 1);
  std::vector<CellIndex> planning_polygon_map;
  int32_t mx, my;
  for (auto const &pose : planning_polygon.poses) {
    rotated_costmap->worldToMapEnforceBounds(pose.pose.position.x,
                                             pose.pose.position.y, mx, my);
    planning_polygon_map.emplace_back(CellIndex{my, mx, 0});
  }

  //构建一个跟map分辨率等大的cell
  CellIndex temp_cell{my_of_cell, mx_of_cell, 0};
  return in_polygon_cell(temp_cell, planning_polygon_map);
}

auto FreeSpaceCalculator::in_line_cell(CellIndex const &cell, int x0, int y0,
                                       int x1, int y1) -> bool {
  using namespace std;

  auto px = cell.row;
  auto py = cell.col;

  if (px > max(x0, x1) || px < min(x0, x1) || py > max(y0, y1) ||
      py < min(y0, y1))
    return false;

  auto side = (py - y0) * (x1 - x0) - (px - x0) * (y1 - y0);

  return side == 0;
}

auto FreeSpaceCalculator::in_polygon_cell(CellIndex const &cell,
                                          std::vector<CellIndex> const &polygon)
    -> InPolygonState {
  using namespace std;

  auto size = polygon.size();
  if (size < 3)
    return InPolygonState::out_of_polygon;

  auto p_x = cell.row;
  auto p_y = cell.col;
  auto counter = 0;
  double xinters;
  int p1_x, p1_y, p2_x, p2_y;
  p1_x = polygon.back().row;
  p1_y = polygon.back().col;

  for (size_t i = 0; i < size; i++) {
    p2_x = polygon[i].row;
    p2_y = polygon[i].col;

    if (in_line_cell(cell, p1_x, p1_y, p2_x, p2_y)) {
      return InPolygonState::in_border;
    }

    if (p1_y == p2_y) {
      p1_x = p2_x;
      continue;
    }

    if (p_y >= min(p1_y, p2_y) && p_y < max(p1_y, p2_y) &&
        p_x <= max(p1_x, p2_x)) {
      xinters = double((p_y - p1_y) * (p2_x - p1_x)) / (p2_y - p1_y) + p1_x;
      if (p1_x == p2_x || p_x <= xinters) {
        counter++;
      }
    }

    p1_x = p2_x;
    p1_y = p2_y;
  }

  if (counter % 2 != 0)
    return InPolygonState::in_polygon;
  return InPolygonState::out_of_polygon;
}
