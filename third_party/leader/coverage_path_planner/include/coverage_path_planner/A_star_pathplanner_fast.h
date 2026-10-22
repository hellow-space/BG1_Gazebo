
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <queue>
#include <stdio.h>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "node.h"

#pragma once

class AStarFastPlanner {
public:
  AStarFastPlanner() {
    downsample_factor_ = 1.0;
    obs_inflate_ = 0;
    map_resolution_ = 0.05;
  };

  void prepareMap(const cv::Mat &map, double downsample_factor,
                  int obs_inflate, double map_resolution);
  double planPathFast(const cv::Point &start, const cv::Point &goal,
                      int end_radius, std::vector<cv::Point> &out_path);
  double planPathFast(const cv::Point &start, const cv::Point &goal);
  bool isFree(const cv::Point2f &p);
  cv::Point ensureFreePoint(const cv::Point &p);
  bool smoothPath(std::vector<cv::Point> ori_path,
                   std::vector<cv::Point> &path, int smoothness_factor = 512);
 bool smoothPath2(std::vector<cv::Point> ori_path,
                   std::vector<cv::Point> &path, int smoothness_factor = 512);
 void smoothPath3(const std::vector<cv::Point>& raw_path, std::vector<cv::Point>& smoothed_path);

private:
  std::string pathFind(int xs, int ys, int xe, int ye);
  std::string getRouteWithNeighborhood(int xs, int ys, int xe, int ye,
                                       int radius);
  bool isLineFree(int x1, int y1, int x2, int y2);

private:
  double downsample_factor_;
  int obs_inflate_;
  double map_resolution_;

  cv::Mat ds_map_;
  cv::Mat obstacle_;
  cv::Mat closed_;
  cv::Mat open_;
  cv::Mat parent_dir_;

  int rows_ = 0;
  int cols_ = 0;

  std::priority_queue<NodeAstar> pq_[2];
};