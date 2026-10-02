cmake_minimum_required(VERSION 3.16)
project(mobile_aloha_mujoco_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME piper_geometry
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_piper_geometry.py)
  add_test(
    NAME vx300s_gripper_geometry
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vx300s_gripper_geometry.py)
endif()

install(DIRECTORY config mjcf urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY LICENSES DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

ament_package()
