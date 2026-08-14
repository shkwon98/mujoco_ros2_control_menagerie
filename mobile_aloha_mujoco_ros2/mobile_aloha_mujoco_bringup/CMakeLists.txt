cmake_minimum_required(VERSION 3.16)
project(mobile_aloha_mujoco_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
