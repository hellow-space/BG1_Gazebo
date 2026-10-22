#!/usr/bin/env python3
"""Generate a ROS 2 build copy. Never write to leader-supplied sources."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def replace_body(text, signature, body):
    start=text.index(signature)
    opening=text.index('{',start);depth=1;end=opening+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[:opening+1]+'\n'+body+'\n'+text[end-1:]


def prepare(source,dest):
    dest.mkdir(parents=True,exist_ok=True);hashes={}
    files=list((source/'include/coverage_path_planner').glob('*.h'))+list((source/'src').glob('*.cpp'))
    for path in files:
        if path.name=='PathPlanningNode.h':continue
        text=path.read_text();hashes[str(path.relative_to(source))]=hashlib.sha256(path.read_bytes()).hexdigest()
        text=re.sub(r'^#include [<"](?:robot_msg/[^>"\n]+|tf/[^>"\n]+|costmap_2d/costmap_2d_ros.h)[>"]\s*$', '',text,flags=re.M)
        includes={'nav_msgs/Path.h':'nav_msgs/msg/path.hpp',
                  'nav_msgs/OccupancyGrid.h':'nav_msgs/msg/occupancy_grid.hpp',
                  'geometry_msgs/PoseStamped.h':'geometry_msgs/msg/pose_stamped.hpp',
                  'geometry_msgs/Vector3.h':'geometry_msgs/msg/vector3.hpp',
                  'costmap_2d/cost_values.h':'nav2_costmap_2d/cost_values.hpp',
                  'costmap_2d/costmap_2d.h':'nav2_costmap_2d/costmap_2d.hpp','ros/ros.h':'compat.hpp'}
        for old,new in includes.items():text=text.replace(old,new)
        text=text.replace('nav_msgs::','nav_msgs::msg::').replace('geometry_msgs::','geometry_msgs::msg::')
        text=text.replace('costmap_2d::','nav2_costmap_2d::').replace('ros::Time::now()','rclcpp::Clock().now()').replace('ros::ok()','rclcpp::ok()')
        if path.name=='task_helper.h':
            text=text.replace('TaskHelper() { ros::NodeHandle nh_("~"); }','TaskHelper() = default;')
            text=re.sub(r'  bool assessmentCleanTask\([^;]+;', '',text,flags=re.S)
            text=re.sub(r'  float getRegionCleanTimeRequire\([^;]+;', '',text,flags=re.S)
        if path.name=='task_helper.cpp':
            # Optional proprietary JSON/device loading is outside this preview.
            text=replace_body(text,'TaskHelper::initPlanTask(','''  debug_=debug;
  size_x_=grid.info.width; size_y_=grid.info.height;
  resolution_=grid.info.resolution;
  origin_x_=grid.info.origin.position.x; origin_y_=grid.info.origin.position.y;
  regions_.clear(); is_docker_exist_=false;
  world_costmap_ptr_=grid2Costmap(grid);
  costmat=costmap2Costmat(world_costmap_ptr_);
  return world_costmap_ptr_;''')
            for signature in ('bool TaskHelper::assessmentCleanTask(', 'float TaskHelper::getRegionCleanTimeRequire('):
                begin=text.index(signature);opening=text.index('{',begin);depth=1;end=opening+1
                while depth:
                    depth+=(text[end]=='{')-(text[end]=='}');end+=1
                text=text[:begin]+text[end:]
        if path.name=='BezierTrajectoryGeneratorWaypoint.cpp':
            # Original one-pose input dereferences begin()-1 when computing yaw.
            text=text.replace('if (path.poses.empty()) {','if (path.poses.empty()) {')
            text=text.replace('  auto ret_path(path);','  if (path.poses.size()==1) return path;\n  auto ret_path(path);')
        (dest/path.name).write_text('// Build-generated ROS 2 adaptation; source provenance in source_hashes.json\n'+text)
    (dest/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('dest',type=Path)
    args=parser.parse_args();prepare(args.source,args.dest)
