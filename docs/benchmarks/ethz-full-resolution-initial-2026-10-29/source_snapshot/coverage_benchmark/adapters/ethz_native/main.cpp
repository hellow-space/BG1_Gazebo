// Headless I/O only; decomposition, sweeps, visibility and GK-MA are upstream.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/mono-config.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_planners/planners/polygon_stripmap_planner.h>
#include <polygon_coverage_planners/sensor_models/line.h>

int main(int argc,char** argv) {
    using namespace polygon_coverage_planning;
    try {
        const char* base=std::getenv("ETHZ_BUILD_ROOT");
        if(!base) throw std::runtime_error("ETHZ_BUILD_ROOT missing");
        const std::string assembly_dir=std::string(base)+"/deps/usr/lib";
        const std::string config_dir=std::string(base)+"/deps/etc";
        mono_set_dirs(assembly_dir.c_str(),config_dir.c_str());
        mono_config_parse(nullptr);
        if(argc!=3) throw std::runtime_error("usage: ethz_planner input.txt output.txt");
        std::ifstream in(argv[1]);
        std::string magic; size_t rings; double spacing,sx,sy,gx,gy;
        if(!(in>>magic>>spacing>>sx>>sy>>gx>>gy>>rings) || magic!="ETHZ1" || !rings || spacing<=0)
            throw std::runtime_error("invalid input header");
        std::vector<Polygon_2> polygons;
        for(size_t r=0;r<rings;++r) {
            size_t n; if(!(in>>n) || n<3) throw std::runtime_error("invalid ring");
            Polygon_2 p;
            for(size_t i=0;i<n;++i) { double x,y; if(!(in>>x>>y)) throw std::runtime_error("invalid point"); p.push_back(Point_2(x,y)); }
            simplifyPolygon(&p);
            if(!p.is_simple() || p.area()==0) throw std::runtime_error("non-simple or zero-area ring");
            if((r==0 && p.is_clockwise_oriented()) || (r>0 && p.is_counterclockwise_oriented())) p.reverse_orientation();
            polygons.push_back(p);
        }
        PolygonWithHoles polygon(polygons[0],polygons.begin()+1,polygons.end());
        if(!isStrictlySimple(polygon) || !pointInPolygon(polygon,Point_2(sx,sy)) || !pointInPolygon(polygon,Point_2(gx,gy)))
            throw std::runtime_error("invalid polygon or start/goal outside input");
        sweep_plan_graph::SweepPlanGraph::Settings settings;
        settings.polygon=polygon;
        settings.cost_function=computeEuclideanPathCost;
        settings.sensor_model=std::make_shared<Line>(spacing,0.0);
        settings.decomposition_type=DecompositionType::kBCD;
        settings.wall_distance=0.0; // Input is already a clearance-safe polygon.
        settings.offset_polygons=false; // Match official coverage_planner.yaml.
        settings.sweep_single_direction=true; // Native best-direction mode for raster complexity.
        PolygonStripmapPlanner planner(settings);
        std::cerr<<"ETHZ_STAGE setup\n"<<std::flush;
        if(!planner.setup()) throw std::runtime_error("native setup failed");
        auto cells=planner.getDecomposition();
        if(cells.empty()) throw std::runtime_error("native decomposition empty");
        std::cerr<<"ETHZ_STAGE solve cells="<<cells.size()<<"\n"<<std::flush;
        std::vector<Point_2> path;
        if(!planner.solve(Point_2(sx,sy),Point_2(gx,gy),&path) || path.size()<2)
            throw std::runtime_error("native solve failed or empty path");
        std::ofstream out(argv[2]); out<<std::setprecision(17);
        out<<"ETHZ1 "<<cells.size()<<' '<<path.size()<<'\n';
        for(const auto& p:path) out<<CGAL::to_double(p.x())<<' '<<CGAL::to_double(p.y())<<'\n';
        if(!out) throw std::runtime_error("cannot write output");
        return 0;
    } catch(const std::exception& error) {std::cerr<<"ETHZ_ERROR "<<error.what()<<'\n';return 2;}
}
