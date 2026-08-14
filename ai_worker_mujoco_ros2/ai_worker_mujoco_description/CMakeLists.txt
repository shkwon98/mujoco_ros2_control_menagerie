cmake_minimum_required(VERSION 3.8)
project(ai_worker_mujoco_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description_split test/test_description_split.py)
endif()

install(
  DIRECTORY config meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE.robotis_mujoco_menagerie
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
