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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME hand2_composition
    COMMAND
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hand2.py)
endif()

install(
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../README.md" LICENSE.wuji_description
  DESTINATION share/${PROJECT_NAME})

ament_package()
