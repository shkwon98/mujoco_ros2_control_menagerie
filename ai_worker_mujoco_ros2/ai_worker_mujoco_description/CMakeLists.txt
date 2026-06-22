cmake_minimum_required(VERSION 3.8)
project(ai_worker_mujoco_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE.robotis_mujoco_menagerie
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
