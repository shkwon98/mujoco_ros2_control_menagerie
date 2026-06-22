cmake_minimum_required(VERSION 3.8)
project(rby1_mujoco_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
            meshes
            urdf
            mjcf
  DESTINATION share/${PROJECT_NAME})

ament_package()
