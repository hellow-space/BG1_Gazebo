#include "context.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <ipa_room_exploration/boustrophedon_explorator.h>
#include <ipa_room_exploration/energy_functional_explorator.h>
using json=nlohmann::json;
cv::Mat hybrid_allowed,hybrid_task,hybrid_covered,hybrid_inverse;
int hybrid_free_nodes=0,hybrid_target_nodes=0,hybrid_seed_nodes=0,hybrid_outside_visited=0;
class BCDKernel: public BoustrophedonExplorer {
 public: using BoustrophedonExplorer::computeCellDecompositionWithRotation;
 using BoustrophedonExplorer::computeBoustrophedonPath;
};
static json point_path(const std::vector<cv::Point2f>& points,const cv::Mat& inverse,double r) {
 std::vector<cv::Point2f> original;
 if(!points.empty())cv::transform(points,original,inverse);
 json out=json::array();for(const auto& p:original)out.push_back({(p.x+.5)*r,(p.y+.5)*r});return out;
}
int main(int argc,char** argv) {
 try {
  if(argc!=3)throw std::runtime_error("Expected request.json result.json");
  std::ifstream input(argv[1]);json req;input>>req;
  const double r=req.at("resolution"),width=req.at("tool_width");
  if(!std::isfinite(r)||r<=0||!std::isfinite(width)||width<=0||std::floor(width/std::sqrt(2)/r)<1)throw std::runtime_error("Invalid resolution/tool width");
  hybrid_allowed=cv::imread(req.at("allowed").get<std::string>(),cv::IMREAD_GRAYSCALE);
  if(hybrid_allowed.empty()||!cv::countNonZero(hybrid_allowed))throw std::runtime_error("Empty F map");
  const cv::Point start(std::floor(req["start"][0].get<double>()/r),std::floor(req["start"][1].get<double>()/r));
  const std::string mode=req.at("mode");json result;
  if(mode=="decompose") {
   BCDKernel kernel;cv::Mat R,rotated;cv::Rect bbox;
   std::vector<GeneralizedPolygon> cells;std::vector<cv::Point> centers;
   kernel.computeCellDecompositionWithRotation(hybrid_allowed,r,0.,0,0.,R,bbox,rotated,cells,centers);
   cv::Mat inverse;cv::invertAffineTransform(R,inverse);
   result["cells"]=json::array();int id=0;
   for(const auto& cell:cells) {
    std::vector<cv::Point2f> vertices;for(auto p:cell.getVertices())vertices.push_back(cv::Point2f(p.x,p.y));
    cv::Point robot=cell.getCenter();std::vector<cv::Point2f> path;
    const int spacing=std::floor(width/std::sqrt(2)/r),half=std::floor(width/std::sqrt(2)/r*.5);
    kernel.computeBoustrophedonPath(rotated,r,cell,path,robot,spacing,half,1.,0,0);
    result["cells"].push_back({{"id",id++},{"polygon",point_path(vertices,inverse,r)},
      {"path",point_path(path,inverse,r)},{"source","IPA BoustrophedonExplorer"},{"area_m2",cell.getArea()*r*r}});
   }
   result["diagnostics"]={{"cell_count",cells.size()},{"grid_spacing_pixels",std::floor(width/std::sqrt(2)/r)}};
  } else if(mode=="local") {
   hybrid_task=cv::imread(req.at("task").get<std::string>(),cv::IMREAD_GRAYSCALE);
   hybrid_covered=cv::imread(req.at("covered").get<std::string>(),cv::IMREAD_GRAYSCALE);
   if(hybrid_task.size()!=hybrid_allowed.size()||hybrid_covered.size()!=hybrid_allowed.size())throw std::runtime_error("F/R/C shapes differ");
   if(req.value("target_policy",std::string("centres"))=="disk_gain") {
    // R contains original free residual pixels, potentially outside F. Move only
    // in F: activate centres whose disk can add R minus C, even if the centre is in C.
    cv::Mat residual;cv::bitwise_not(hybrid_covered,residual);cv::bitwise_and(residual,hybrid_task,residual);
    const int radius=std::ceil(width/(2*r));cv::Mat disk=cv::Mat::zeros(2*radius+1,2*radius+1,CV_8U);
    for(int y=-radius;y<=radius;++y)for(int x=-radius;x<=radius;++x)
     if((x*x+y*y)*r*r<=width*width/4+1e-12)disk.at<uchar>(y+radius,x+radius)=255;
    cv::dilate(residual,hybrid_task,disk);cv::bitwise_and(hybrid_task,hybrid_allowed,hybrid_task);
    hybrid_covered.setTo(0,hybrid_task);
   }
   EnergyFunctionalExplorator kernel;std::vector<geometry_msgs::Pose2D> path;Eigen::Matrix<float,2,1> zero;zero<<0,0;
   kernel.getExplorationPath(hybrid_allowed,path,r,start,cv::Point2d(.5*r,.5*r),width/std::sqrt(2)/r,true,zero);
   result["path"]=json::array();for(auto p:path)result["path"].push_back({p.x,p.y});
   result["diagnostics"]={{"free_nodes",hybrid_free_nodes},{"initial_target_nodes",hybrid_target_nodes},
    {"seed_visited_nodes",hybrid_seed_nodes},{"outside_task_visited_nodes",hybrid_outside_visited}};
  } else throw std::runtime_error("Unknown Hybrid native mode");
  std::ofstream output(argv[2]);output<<result.dump()<<'\n';if(!output)throw std::runtime_error("Cannot write output");return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
