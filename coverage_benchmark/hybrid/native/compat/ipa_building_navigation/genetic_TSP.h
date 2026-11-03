#pragma once
#include <stdexcept>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ipa_building_navigation/A_star_pathplanner.h>
class GeneticTSPSolver { public:
 template<class... Args> std::vector<int> solveGeneticTSP(Args&&...) { throw std::runtime_error("Unused IPA TSP interface"); }
};
