cmake_minimum_required(VERSION 3.16)
project(rb_mujoco_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(mujoco_vendor REQUIRED)
  add_executable(test_rb_hold test/test_hold.cpp)
  target_compile_features(test_rb_hold PRIVATE cxx_std_20)
  target_link_libraries(test_rb_hold PRIVATE mujoco::mujoco)
  add_test(
    NAME rb_hold
    COMMAND
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hold.py
      $<TARGET_FILE:test_rb_hold>)
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
install(
  FILES LICENSE.rb_description LICENSE.wuji_description
  DESTINATION share/${PROJECT_NAME})

ament_package()
