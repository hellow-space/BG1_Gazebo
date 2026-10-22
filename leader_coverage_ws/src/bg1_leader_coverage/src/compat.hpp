#pragma once
#include <optional>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
// Source-only custom display colours, unused by planning decisions.
namespace nav2_costmap_2d {
constexpr unsigned char INSCRIBED_PLAN_OBSTACLE=252;
constexpr unsigned char INSCRIBED_POSSIBLY_OBSTACLE=251;
}
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("leader_core"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("leader_core"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("leader_core"), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("leader_core"), __VA_ARGS__)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("leader_core"), x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("leader_core"), x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(rclcpp::get_logger("leader_core"), x)
#define ROS_DEBUG_STREAM(x) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("leader_core"), x)
