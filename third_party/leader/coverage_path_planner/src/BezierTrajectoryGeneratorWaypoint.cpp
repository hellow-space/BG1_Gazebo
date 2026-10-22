#include "BezierTrajectoryGeneratorWaypoint.h"

namespace ikitbot::planning {
auto BezierTrajectoryGeneratorWaypoint::calculate_optimized_path(
    nav_msgs::Path const &path) const -> nav_msgs::Path {
  auto path_size = path.poses.size();
  int size_of_fine_path;
  std::vector<Bezier::Vec2> poses;
  auto control_points = calculate_control_points(path);
  switch (path_size) {
  case 0:
  case 1: {
    return path;
  }

  case 2: {
    Bezier::Bezier<1> bezier(control_points);
    size_of_fine_path =
        std::max(static_cast<int>(bezier.length() / step_in_fine_path_), 2);
    for (auto i = 0; i < size_of_fine_path; i++) {
      poses.emplace_back(bezier.valueAt(
          static_cast<float>(i * 1.0 / (size_of_fine_path - 1))));
    }

    break;
  }
#if 0
  case 3: {
    Bezier::Bezier<2> bezier(control_points);
    size_of_fine_path =
        std::max(static_cast<int>(bezier.length() / step_in_fine_path_), 2);
    for (auto i = 0; i < size_of_fine_path; i++) {
      poses.emplace_back(bezier.valueAt(
          static_cast<float>(i * 1.0 / (size_of_fine_path - 1))));
    }

    break;
  }

  case 4: {
    Bezier::Bezier<3> bezier(control_points);
    size_of_fine_path =
        std::max(static_cast<int>(bezier.length() / step_in_fine_path_), 2);
    for (auto i = 0; i < size_of_fine_path; i++) {
      poses.emplace_back(bezier.valueAt(
          static_cast<float>(i * 1.0 / (size_of_fine_path - 1))));
    }

    break;
  }

  default: {
    Bezier::Bezier<4> bezier(control_points);
    size_of_fine_path =
        std::max(static_cast<int>(bezier.length() / step_in_fine_path_), 2);
    for (auto i = 0; i < size_of_fine_path; i++) {
      poses.emplace_back(bezier.valueAt(
          static_cast<float>(i * 1.0 / (size_of_fine_path - 1))));
    }

    if (path_size > 5) {
      ROS_INFO("There are %lu points in a corner. They are: ", path_size);
      for (auto const &pose : path.poses) {
        ROS_INFO("[%f, %f]", pose.pose.position.x, pose.pose.position.y);
      }
    }
  }
  #else
  default: {
   // ===== 强制使用二阶 Bezier：只取 3 个控制点 =====
    std::vector<Bezier::Point> cps;
    // 起点
    cps.emplace_back(control_points.front());
    // 中间点（拐角）
    cps.emplace_back(control_points[control_points.size() / 2]);
    // 终点
    cps.emplace_back(control_points.back());
    Bezier::Bezier<2> bezier(cps);
    size_of_fine_path =
        std::max(static_cast<int>(bezier.length() / step_in_fine_path_), 2);
    for (auto i = 0; i < size_of_fine_path; i++) {
      poses.emplace_back(bezier.valueAt(
          static_cast<float>(i * 1.0 / (size_of_fine_path - 1))));
    }
    break;
  }
  #endif 

  }

  nav_msgs::Path optimized_path;
  geometry_msgs::PoseStamped pose_stamped;
  for (auto const &pos : poses) {
    pose_stamped.pose.position.x = pos[0];
    pose_stamped.pose.position.y = pos[1];
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.w = 1;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    optimized_path.poses.emplace_back(pose_stamped);
  }

  /**orientation**/
  auto it = optimized_path.poses.begin();
  for (; it < optimized_path.poses.end() - 1; it++) {
    auto theta = atan2((*(it + 1)).pose.position.y - (*it).pose.position.y,
                       (*(it + 1)).pose.position.x - (*it).pose.position.x);
    (*it).pose.orientation.w = cos(theta / 2);
    (*it).pose.orientation.z = sin(theta / 2);
  }

  (*it).pose.orientation = (*(it - 1)).pose.orientation;
  auto ret_optimized_path = calculate_path_orientation(optimized_path);
  return ret_optimized_path.value();
}

auto BezierTrajectoryGeneratorWaypoint::calculate_control_points(
    nav_msgs::Path const &path) const -> std::vector<Bezier::Point> {
  std::vector<Bezier::Point> ret_vec;
  auto path_size = path.poses.size();
  for (size_t i = 0; i < path_size; ++i) {
    if (i < 2 || i == path_size / 2 || i >= path_size - 2) {
      ret_vec.emplace_back(
          Bezier::Point(static_cast<float>(path.poses.at(i).pose.position.x),
                        static_cast<float>(path.poses.at(i).pose.position.y)));
    }
  }

  return ret_vec;
}
auto BezierTrajectoryGeneratorWaypoint::calculate_control_points2(
    nav_msgs::Path const &path) const -> std::vector<Bezier::Point> {
  std::vector<Bezier::Point> ret_vec;
#if 0
  auto n = path.poses.size();
  if (n < 3) {
    for (auto const &p : path.poses)
      ret_vec.emplace_back(p.pose.position.x, p.pose.position.y);
    return ret_vec;
  }

  const double shrink_ratio = 0.3;  // 控制“弯多紧”：0.2~0.4
  for (size_t i = 0; i < n; ++i) {
    const auto &pi = path.poses[i].pose.position;
    // 端点：直接保留
    if (i == 0 || i == n - 1) {
      ret_vec.emplace_back(pi.x, pi.y);
      continue;
    }
    // 前后点
    const auto &p_prev = path.poses[i - 1].pose.position;
    const auto &p_next = path.poses[i + 1].pose.position;
    Eigen::Vector2d v1(pi.x - p_prev.x, pi.y - p_prev.y);
    Eigen::Vector2d v2(p_next.x - pi.x, p_next.y - pi.y);
    if (v1.norm() < 1e-6 || v2.norm() < 1e-6) {
      ret_vec.emplace_back(pi.x, pi.y);
      continue;
    }

    v1.normalize();
    v2.normalize();
    // 判断是否是拐角（方向发生明显变化）
    double cos_angle = v1.dot(v2);
    if (cos_angle < 0.95) {  //  有明显转向
      // 沿前后方向收缩控制点
      Eigen::Vector2d offset = (v1 - v2).normalized();
      double d = shrink_ratio * std::min(
          (Eigen::Vector2d(p_prev.x - pi.x, p_prev.y - pi.y)).norm(),
          (Eigen::Vector2d(p_next.x - pi.x, p_next.y - pi.y)).norm());

      Eigen::Vector2d cp(pi.x, pi.y);
      cp -= offset * d;

      ret_vec.emplace_back(cp.x(), cp.y());
    } else {
      // 直线段，原样保留
      ret_vec.emplace_back(pi.x, pi.y);
    }
  }
#endif
  return ret_vec;
}


auto BezierTrajectoryGeneratorWaypoint::calculate_path_orientation(
    nav_msgs::Path const &path, bool loop_back)
    -> std::optional<nav_msgs::Path> {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  auto ret_path(path);
  auto it = ret_path.poses.begin();
  for (; it < ret_path.poses.end() - 1; it++) {
    calculate_direction(*(it + 1), *it);
  }

  if (!loop_back) {
    (*it).pose.orientation = (*(it - 1)).pose.orientation;
    return ret_path;
  }

  calculate_direction(ret_path.poses.front(), *it);
  return ret_path;
}
} // namespace ikitbot::planning
