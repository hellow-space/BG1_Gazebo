#include "adapter.hpp"
class PlannerServer:public rclcpp::Node{
 public:
 PlannerServer():Node("leader_coverage_planner"){
  map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){map_=msg;});
  service_=create_service<leader_preview::Service>("/leader_coverage/plan",
    [this](const std::shared_ptr<leader_preview::Service::Request> req,std::shared_ptr<leader_preview::Service::Response> out){
      *out=leader_preview::plan(map_?*map_:nav_msgs::msg::OccupancyGrid(),*req);
      auto stamp=now();for(auto path:{&out->coverage_path,&out->boundary_path,&out->island_path}){
        path->header.stamp=stamp;for(auto &pose:path->poses)pose.header=path->header;
      }
      if(out->success)RCLCPP_INFO(get_logger(),"Coverage preview: %zu poses",out->coverage_path.poses.size());
      else RCLCPP_WARN(get_logger(),"Coverage request rejected: %s",out->message.c_str());
    });
 }
 private:
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
 rclcpp::Service<leader_preview::Service>::SharedPtr service_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PlannerServer>());rclcpp::shutdown();}
