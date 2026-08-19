cmake_minimum_required(VERSION 3.16)
project(ai_worker_mujoco_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
