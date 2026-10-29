cmake_minimum_required(VERSION 3.16)
project(bg1_ethz_headless LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -DNDEBUG")
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
set(UPSTREAM "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/polygon_coverage_planning")
set(DEPS "${CMAKE_BINARY_DIR}/../deps/usr")
include_directories(compat "${DEPS}/include/mono-2.0")
foreach(part geometry planners solvers)
  include_directories("${UPSTREAM}/polygon_coverage_${part}/include")
endforeach()
file(GLOB GEOMETRY "${UPSTREAM}/polygon_coverage_geometry/src/*.cc")
# CGAL 5.6 defaults the offset output to Epick; preserve upstream Epeck explicitly.
# Only this generated build copy changes, not the pinned source.
list(REMOVE_ITEM GEOMETRY "${UPSTREAM}/polygon_coverage_geometry/src/offset.cc")
file(READ "${UPSTREAM}/polygon_coverage_geometry/src/offset.cc" OFFSET_SOURCE)
string(REPLACE ", sorted_pwh);" ", sorted_pwh, K());" OFFSET_SOURCE "${OFFSET_SOURCE}")
file(WRITE "${CMAKE_BINARY_DIR}/offset_cgal56.cc" "${OFFSET_SOURCE}")
list(APPEND GEOMETRY "${CMAKE_BINARY_DIR}/offset_cgal56.cc")
add_executable(ethz_planner main.cpp ${GEOMETRY}
  "${UPSTREAM}/polygon_coverage_planners/src/cost_functions/path_cost_functions.cc"
  "${UPSTREAM}/polygon_coverage_planners/src/graphs/sweep_plan_graph.cc"
  "${UPSTREAM}/polygon_coverage_planners/src/planners/polygon_stripmap_planner.cc"
  "${UPSTREAM}/polygon_coverage_planners/src/timing.cc"
  "${UPSTREAM}/polygon_coverage_solvers/src/gk_ma.cc")
target_include_directories(ethz_planner PRIVATE "${DEPS}/include" "${DEPS}/include/x86_64-linux-gnu")
target_link_libraries(ethz_planner CGAL::CGAL Eigen3::Eigen "${DEPS}/lib/libmonosgen-2.0.so" pthread dl)
set_target_properties(ethz_planner PROPERTIES BUILD_RPATH "${DEPS}/lib")
