cmake_minimum_required(VERSION 3.8)
project(ai_worker_mujoco_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
