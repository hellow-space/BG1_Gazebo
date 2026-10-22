#include "adapter.hpp"
#include "CoveragePathPlanner.h"
#include "task_helper.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace leader_preview {
namespace {
using P=geometry_msgs::msg::Point32;
double cross(const P&a,const P&b,const P&c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
bool on(const P&a,const P&b,const P&p){return std::abs(cross(a,b,p))<1e-7&&p.x>=std::min(a.x,b.x)-1e-7&&p.x<=std::max(a.x,b.x)+1e-7&&p.y>=std::min(a.y,b.y)-1e-7&&p.y<=std::max(a.y,b.y)+1e-7;}
bool intersects(const P&a,const P&b,const P&c,const P&d){
 return (cross(a,b,c)*cross(a,b,d)<0&&cross(c,d,a)*cross(c,d,b)<0)||on(a,b,c)||on(a,b,d)||on(c,d,a)||on(c,d,b);
}
void require(bool condition,const std::string &message){if(!condition)throw std::invalid_argument(message);}
}
Service::Response plan(const nav_msgs::msg::OccupancyGrid &map,const Service::Request &request){
 Service::Response result;
 try {
  const auto &info=map.info;const auto &points=request.region.polygon.points;
  require(info.width>0&&info.height>0&&info.width<=10000&&info.height<=10000&&
          map.data.size()==size_t(info.width)*info.height&&std::isfinite(info.resolution)&&info.resolution>0,"Map missing or invalid");
  require(map.header.frame_id=="map"&&request.region.header.frame_id=="map","Region and map must use map frame");
  const auto &q=info.origin.orientation;
  require(std::abs(q.x)<1e-6&&std::abs(q.y)<1e-6&&std::abs(q.z)<1e-6&&std::abs(q.w-1)<1e-6,"Rotated map origins are unsupported");
  require(std::isfinite(info.origin.position.x)&&std::isfinite(info.origin.position.y),"Invalid map origin");
  require(request.pattern<=1,"Pattern must be 0 (zigzag) or 1 (backshaped)");
  require(std::isfinite(request.stripe_spacing)&&request.stripe_spacing>=info.resolution&&request.stripe_spacing<=2,"Stripe spacing must be between map resolution and 2 m");
  require(std::isfinite(request.clearance)&&request.clearance>=0.1&&request.clearance<=2,"Clearance must be between 0.1 and 2 m");
  require(points.size()>=3&&points.size()<=100,"Select 3 to 100 distinct polygon vertices");
  auto inside_map=[&](double x,double y){return std::isfinite(x)&&std::isfinite(y)&&x>=info.origin.position.x&&y>=info.origin.position.y&&x<info.origin.position.x+info.width*info.resolution&&y<info.origin.position.y+info.height*info.resolution;};
  std::vector<cv::Point> contour;double area=0;
  for(size_t i=0;i<points.size();++i){
   const auto&a=points[i];const auto&b=points[(i+1)%points.size()];
   require(inside_map(a.x,a.y),"Polygon vertex is nonfinite or outside map");
   require(std::hypot(a.x-b.x,a.y-b.y)>=info.resolution,"Polygon contains duplicate/too-close vertices");
   area+=a.x*b.y-a.y*b.x;
   contour.emplace_back(int((a.x-info.origin.position.x)/info.resolution),int((a.y-info.origin.position.y)/info.resolution));
   for(size_t j=i+1;j<points.size();++j){
    if(j==i+1||(i==0&&j==points.size()-1))continue;
    require(!intersects(a,b,points[j],points[(j+1)%points.size()]),"Polygon crosses or touches itself");
   }
  }
  require(std::abs(area)>.1,"Selected polygon has no usable area");
  require(inside_map(request.start.x,request.start.y),"Start point is nonfinite or outside map");
  const int sx=int((request.start.x-info.origin.position.x)/info.resolution),sy=int((request.start.y-info.origin.position.y)/info.resolution);
  require(map.data[sy*info.width+sx]==0,"Start point must be in known free space");
  // Explicitly mask outside the selection BEFORE the leader's obstacle inflation.
  // Original planner otherwise searches connecting segments on the full map.
  cv::Mat mask(info.height,info.width,CV_8U,cv::Scalar(0));
  cv::fillPoly(mask,std::vector<std::vector<cv::Point>>{contour},cv::Scalar(255));
  auto selected_map=map;
  for(size_t i=0;i<selected_map.data.size();++i)
    selected_map.data[i]=(mask.data[i]&&map.data[i]==0)?0:100;
  TaskHelper helper;cv::Mat costmat;std::string error;
  auto costmap=helper.initPlanTask("preview",selected_map,costmat,0,false,error);
  CleanRegionInfo region;region.path_type=request.pattern;
  region.zigzag_space=std::max(1,int(std::round(request.stripe_spacing/info.resolution)));
  const int inflation=int(std::ceil(request.clearance/info.resolution));
  region.number_erosions=inflation;
  for(const auto&p:points){int x,y;helper.worldToMap(p.x,p.y,x,y);region.task_cell_vertices.emplace_back(x,y);}
  cv::Mat task;
  require(helper.getTasMatAndBoudPath(costmat,0,0,inflation,region,task,false),"No connected work area remains after clearance inflation");
  ikitbot::planning::CoveragePathPlanner planner;
  // Island tracing is optional in the original and disabled for initial preview.
  auto paths=planner.get_path_interface(task,cv::Point2f(request.start.x,request.start.y),costmap,region,false,2,.3,3,inflation,false);
  require(!paths.empty()&&paths[0].poses.size()>1,"Leader planner returned no coverage path");
  auto valid=[&](double x,double y){
   if(!inside_map(x,y))return false;
   int mx=int((x-info.origin.position.x)/info.resolution),my=int((y-info.origin.position.y)/info.resolution);
   return mask.at<unsigned char>(my,mx)&&task.at<unsigned char>(info.height-1-my,mx)==0;
  };
  // Check full line segments, not just the source's sampled waypoints.
  auto validate_path=[&](nav_msgs::msg::Path &path){
   path.header.frame_id="map";
   for(size_t i=0;i<path.poses.size();++i){
    auto &p=path.poses[i];p.header=path.header;
    require(valid(p.pose.position.x,p.pose.position.y),"Generated path point violates selection/clearance; request rejected");
    if(i){const auto &a=path.poses[i-1].pose.position;const auto &b=p.pose.position;
     int steps=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/(info.resolution*.5))));
     for(int j=1;j<steps;++j)require(valid(a.x+(b.x-a.x)*j/steps,a.y+(b.y-a.y)*j/steps),"Generated connecting segment violates selection/clearance; request rejected");
    }
   }
  };
  result.coverage_path=paths[0];validate_path(result.coverage_path);
  for(const auto &pixel:region.boundary_path){
   float x,y;helper.mapToWorld(pixel.x,pixel.y,x,y);
   geometry_msgs::msg::PoseStamped p;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1;
   result.boundary_path.poses.push_back(p);
  }
  // Boundary is a closed contour for visualization only.
  if(!result.boundary_path.poses.empty())result.boundary_path.poses.push_back(result.boundary_path.poses.front());
  result.boundary_path.header.frame_id="map";
  result.success=true;result.message="Leader planner preview generated; robot motion is not enabled";
 } catch(const std::exception &e){
  result=Service::Response();result.success=false;result.message=e.what();
 }
 return result;
}
}
