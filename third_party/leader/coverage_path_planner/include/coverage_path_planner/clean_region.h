#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

struct CleanRegionInfo {
  CleanRegionInfo() { Reset(); }
  CleanRegionInfo(const CleanRegionInfo &info) { *this = info; }
  const CleanRegionInfo &operator=(const CleanRegionInfo &info) {
    if (this != &info) {
      Reset();
      parent_region_id = info.parent_region_id;
      region_id = info.region_id;
      region_name = info.region_name;
      path_type = info.path_type;
      mode = info.mode;
      line_speed = info.line_speed;
      start = info.start;
      zigzag_space = info.zigzag_space;
      number_erosions = info.number_erosions;
      first_path_space = info.first_path_space;
      path_to_obs_dis = info.path_to_obs_dis;
      min_line_length = info.min_line_length;
      clean_area = info.clean_area;
      time_require = info.time_require;

      // 遍历复制 task_cell_vertices_
      for (const auto &p : info.task_cell_vertices) {
        task_cell_vertices.push_back(p);
      }

      // 遍历复制 contours_vertices_
      for (const auto &p : info.contours_vertices) {
        contours_vertices.push_back(p);
      }

      // 遍历复制 coverage_path
      for (const auto &p : info.coverage_path) {
        coverage_path.push_back(p);
      }

      // 遍历复制 boundary_path
      for (const auto &p : info.boundary_path) {
        boundary_path.push_back(p);
      }
    }
    return *this;
  }
  void Reset() {
    parent_region_id = -1;
    region_id = 0;
    path_type = 0;
    region_name = "";
    mode = 0;
    line_speed = 0.0f;
    start.x = 0.0f;
    start.y = 0.0f;
    zigzag_space = 8;
    number_erosions = 3;
    first_path_space = 3;
    path_to_obs_dis = 2;
    min_line_length = 20;
    clean_area = 0.0f;
    time_require = 0.0f;
    task_cell_vertices.clear();
    contours_vertices.clear();
    coverage_path.clear();
    boundary_path.clear();
  }

  int parent_region_id = -1;
  int region_id;
  int path_type = 0;
  std::string region_name;
  int mode;
  float line_speed;
  cv::Point start;
  int zigzag_space = 8;
  int number_erosions = 3;
  int first_path_space = 3;
  int path_to_obs_dis = 2;
  int min_line_length = 20;
  float clean_area = 0.0f;
  float time_require = 0.0f;
  std::vector<cv::Point> task_cell_vertices;
  std::vector<cv::Point> contours_vertices;
  std::vector<cv::Point> coverage_path;
  std::vector<cv::Point> boundary_path;
};
