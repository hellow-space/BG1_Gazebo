#include <gtest/gtest.h>
#include "adapter.hpp"

using Request=bg1_leader_coverage_interfaces::srv::PlanCoverage::Request;
nav_msgs::msg::OccupancyGrid test_map(){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";
  m.info.resolution=.05;m.info.width=240;m.info.height=200;
  m.info.origin.orientation.w=1;m.data.assign(240*200,0);
  // One square island; the planner must not cut through it.
  for(int y=80;y<120;++y)for(int x=100;x<140;++x)m.data[y*240+x]=100;
  return m;
}
Request request(){
  Request r;r.region.header.frame_id="map";
  for(auto [x,y]:std::vector<std::pair<float,float>>{{.2,.2},{11.8,.2},{11.8,9.8},{.2,9.8}}){
    geometry_msgs::msg::Point32 p;p.x=x;p.y=y;r.region.polygon.points.push_back(p);
  }
  r.start.x=2;r.start.y=2;r.stripe_spacing=.7;r.clearance=.5;
  return r;
}
TEST(Adapter, RejectsMissingOrInvalidMap){
  auto r=request();EXPECT_FALSE(leader_preview::plan(nav_msgs::msg::OccupancyGrid(),r).success);
  auto m=test_map();m.data.pop_back();EXPECT_FALSE(leader_preview::plan(m,r).success);
}
TEST(Adapter, RejectsInvalidRegionAndSettings){
  auto m=test_map();auto r=request();r.region.header.frame_id="odom";
  EXPECT_FALSE(leader_preview::plan(m,r).success);
  r=request();r.stripe_spacing=0;EXPECT_FALSE(leader_preview::plan(m,r).success);
  r=request();r.pattern=2;EXPECT_FALSE(leader_preview::plan(m,r).success);
  r=request();std::swap(r.region.polygon.points[1],r.region.polygon.points[2]);
  EXPECT_FALSE(leader_preview::plan(m,r).success);
  r=request();r.start.x=6;r.start.y=5;EXPECT_FALSE(leader_preview::plan(m,r).success);
  r=request();r.region.polygon.points[0].x=-1;EXPECT_FALSE(leader_preview::plan(m,r).success);
  m.data.assign(m.data.size(),-1);EXPECT_FALSE(leader_preview::plan(m,request()).success);
}
TEST(Adapter, BothLeaderPatternsPlanAroundIsland){
  auto m=test_map();auto r=request();
  for(int pattern=0;pattern<2;++pattern){
    r.pattern=pattern;auto result=leader_preview::plan(m,r);
    ASSERT_TRUE(result.success)<<result.message;
    ASSERT_GT(result.coverage_path.poses.size(),20u);
    for(const auto &p:result.coverage_path.poses){
      EXPECT_TRUE(std::isfinite(p.pose.position.x));
      double dx=std::max({5.-p.pose.position.x,0.,p.pose.position.x-7.});
      double dy=std::max({4.-p.pose.position.y,0.,p.pose.position.y-6.});
      // Pixel-centre rasterization allows at most a half-cell diagonal error.
      EXPECT_GE(std::hypot(dx,dy),.5-.05/std::sqrt(2.));
    }
  }
}
int main(int argc,char **argv){
 rclcpp::init(argc,argv);testing::InitGoogleTest(&argc,argv);auto result=RUN_ALL_TESTS();rclcpp::shutdown();return result;
}

TEST(Adapter, InflatesPartiallyOccupiedCellsAsObstacles){
 auto m=test_map();for(auto &value:m.data)if(value==100)value=50;
 auto r=request();auto result=leader_preview::plan(m,r);
 ASSERT_TRUE(result.success)<<result.message;
 for(const auto &p:result.coverage_path.poses){
   double dx=std::max({5.-p.pose.position.x,0.,p.pose.position.x-7.});
   double dy=std::max({4.-p.pose.position.y,0.,p.pose.position.y-6.});
   EXPECT_GE(std::hypot(dx,dy),.5-.05/std::sqrt(2.));
 }
}
