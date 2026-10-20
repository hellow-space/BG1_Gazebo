cmake_minimum_required(VERSION 3.8)
project(bg1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch materials meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS tools/build_visual_assets.py DESTINATION lib/${PROJECT_NAME})
install(FILES tools/mesh_primitives.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(configuration_contract test/test_configuration.py)
  ament_add_pytest_test(model_contract test/test_model_contract.py)
  ament_add_pytest_test(appearance_contract test/test_appearance_contract.py)
endif()

ament_package()
