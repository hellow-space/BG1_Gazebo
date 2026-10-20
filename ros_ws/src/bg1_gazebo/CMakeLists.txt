cmake_minimum_required(VERSION 3.8)
project(bg1_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/verify_runtime.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_contract test/test_launch_contract.py)
  ament_add_pytest_test(runtime_helpers test/test_runtime_helpers.py)
  ament_add_pytest_test(navigation_frames test/test_navigation_frames.py)
endif()

ament_package()
