#include "context.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <ipa_room_exploration/boustrophedon_explorator.h>

using json = nlohmann::json;
json bcd_trace={{"events",json::array()},{"sweeps",json::array()},
  {"rotations",json::array()},{"directions",json::array()},{"rows",json::array()}};
std::string bcd_trace_dir;
cv::Mat hybrid_allowed, hybrid_task, hybrid_covered, hybrid_inverse;
int hybrid_free_nodes=0, hybrid_target_nodes=0, hybrid_seed_nodes=0, hybrid_outside_visited=0;

class DemoBCD : public BoustrophedonExplorer {
public:
  using BoustrophedonExplorer::computeCellDecomposition;
};

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("usage: native_demo input.png output_directory");
    bcd_trace_dir=argv[2];
    cv::Mat map=cv::imread(argv[1],cv::IMREAD_GRAYSCALE);
    if (map.empty()) throw std::runtime_error("cannot read input");
    std::vector<GeneralizedPolygon> cells;
    std::vector<cv::Point> centers;
    DemoBCD demo;
    demo.computeCellDecomposition(map,1.0,0.0,0,cells,centers);
    bcd_trace["polygon_count"]=cells.size();
    std::ofstream out(bcd_trace_dir+"/trace.json");
    out << bcd_trace.dump(2) << '\n';
    if (!out) throw std::runtime_error("cannot write trace");
    return 0;
  } catch(const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 2;
  }
}
