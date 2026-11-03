#pragma once
#include <opencv2/opencv.hpp>
#include <limits>
#include <algorithm>
#include <cmath>
#ifndef CV_RETR_CCOMP
#define CV_RETR_CCOMP cv::RETR_CCOMP
#define CV_RETR_EXTERNAL cv::RETR_EXTERNAL
#define CV_CHAIN_APPROX_NONE cv::CHAIN_APPROX_NONE
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#define CV_FILLED cv::FILLED
#define CV_DIST_L2 cv::DIST_L2
#endif
extern cv::Mat hybrid_allowed, hybrid_task, hybrid_covered, hybrid_inverse;
extern int hybrid_free_nodes, hybrid_target_nodes, hybrid_seed_nodes, hybrid_outside_visited;
inline cv::Point hybrid_original(const cv::Point& p) {
 const double x=hybrid_inverse.at<double>(0,0)*p.x+hybrid_inverse.at<double>(0,1)*p.y+hybrid_inverse.at<double>(0,2);
 const double y=hybrid_inverse.at<double>(1,0)*p.x+hybrid_inverse.at<double>(1,1)*p.y+hybrid_inverse.at<double>(1,2);
 return cv::Point(std::floor(x+.5),std::floor(y+.5));
}
inline bool hybrid_mask(const cv::Mat& mask,const cv::Point& p) {
 const cv::Point q=hybrid_original(p);
 return q.x>=0 && q.y>=0 && q.x<mask.cols && q.y<mask.rows && mask.at<uchar>(q.y,q.x)==255;
}
