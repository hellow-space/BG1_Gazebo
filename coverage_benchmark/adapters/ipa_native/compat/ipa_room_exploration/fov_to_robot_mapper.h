#pragma once
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose2D.h>
#define ROS_INFO(...) ((void)0)
// Footprint mode returns before this call. Refuse unsupported sensor mode explicitly.
inline void mapPath(const cv::Mat&, std::vector<geometry_msgs::Pose2D>&,
    const std::vector<geometry_msgs::Pose2D>&, const Eigen::Matrix<float,2,1>&,
    const float, const cv::Point2d, const cv::Point) {
    throw std::runtime_error("IPA headless adapter supports footprint mode only");
}
