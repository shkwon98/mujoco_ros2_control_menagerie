cmake_minimum_required(VERSION 3.8)
project(g1_mujoco_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE.unitree_ros THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
