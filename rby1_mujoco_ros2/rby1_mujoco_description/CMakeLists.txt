cmake_minimum_required(VERSION 3.8)
project(rby1_mujoco_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
            meshes
            urdf
            mjcf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hand2 test/test_hand2.py)
endif()

install(
  FILES LICENSE.wuji_description NOTICE.wuji_description
  DESTINATION share/${PROJECT_NAME})

ament_package()
