cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control_menagerie)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME description_partition
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/../test/test_description_partition.py)
endif()

ament_package()
