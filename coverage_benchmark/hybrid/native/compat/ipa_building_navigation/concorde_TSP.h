#pragma once
#include <stdexcept>
#include <vector>
#include <ipa_building_navigation/A_star_pathplanner.h>
class ConcordeTSPSolver { public:
 template<class... Args> std::vector<int> solveConcordeTSP(Args&&...) { throw std::runtime_error("Unused IPA TSP interface"); }
};
