cmake_minimum_required(VERSION 3.16)
project(mobile_aloha_mujoco_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config mjcf urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY LICENSES DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

ament_package()
