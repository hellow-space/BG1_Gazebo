#include <algorithm>
#include <limits>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <ipa_room_exploration/energy_functional_explorator.h>
int main(int argc,char** argv) {
    try {
        if(argc!=8) throw std::runtime_error("Expected map output resolution start_x start_y coverage_radius");
        const double r=std::stod(argv[3]), sx=std::stod(argv[4]), sy=std::stod(argv[5]), radius=std::stod(argv[6]);
        // argv[7] is an explicit protocol version, preventing accidental CLI mismatch.
        if(std::string(argv[7])!="IPA1" || !std::isfinite(r) || r<=0 || !std::isfinite(radius) || radius<=0
                || !std::isfinite(sx) || !std::isfinite(sy) || std::floor(radius*std::sqrt(2)/r)<1)
            throw std::runtime_error("Invalid IPA parameters");
        cv::Mat image=cv::imread(argv[1],cv::IMREAD_GRAYSCALE);
        if(image.empty() || cv::countNonZero(image)==0) throw std::runtime_error("Empty native map");
        const cv::Point start(std::lround(sx/r-.5),std::lround(sy/r-.5));
        if(start.x<0 || start.y<0 || start.x>=image.cols || start.y>=image.rows || image.at<uchar>(start.y,start.x)!=255)
            throw std::runtime_error("Native start outside allowed map");
        std::vector<geometry_msgs::Pose2D> path;
        Eigen::Matrix<float,2,1> zero;zero<<0,0;
        EnergyFunctionalExplorator planner;
        planner.getExplorationPath(image,path,r,start,cv::Point2d(.5*r,.5*r),radius*std::sqrt(2)/r,true,zero);
        std::ofstream output(argv[2]); output<<"IPA1 "<<path.size()<<'\n'<<std::setprecision(17);
        for(const auto& p:path) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y)) throw std::runtime_error("Nonfinite native pose");
            output<<p.x<<' '<<p.y<<'\n';
        }
        if(!output) throw std::runtime_error("Cannot write complete native output");
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<std::endl;return 2; }
}
