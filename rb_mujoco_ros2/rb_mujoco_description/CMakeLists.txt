cmake_minimum_required(VERSION 3.16)
project(rb_mujoco_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME rb_models
    COMMAND
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_models.py)
  add_test(
    NAME rb_wuji
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wuji.py)
endif()

install(
  DIRECTORY config
            meshes
            mjcf
            urdf
  DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE.rb_description DESTINATION share/${PROJECT_NAME})

ament_package()
