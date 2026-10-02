cmake_minimum_required(VERSION 3.8)
project(ai_worker_mujoco_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME visual_materials
    COMMAND
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_visuals.py)
endif()

install(
  DIRECTORY config
            meshes
            mjcf
            urdf
  DESTINATION share/${PROJECT_NAME})

install(
  FILES LICENSE.robotis_mujoco_menagerie DESTINATION share/${PROJECT_NAME})

ament_package()
