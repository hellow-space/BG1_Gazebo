#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <bg1_leader_coverage_interfaces/srv/plan_coverage.hpp>
namespace leader_preview {
using Service=bg1_leader_coverage_interfaces::srv::PlanCoverage;
Service::Response plan(const nav_msgs::msg::OccupancyGrid &map,const Service::Request &request);
}
