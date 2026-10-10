cmake_minimum_required(VERSION 3.16)
project(rb_mujoco_bringup)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(
  PROGRAMS scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_instances
    test/test_instances.py
    TIMEOUT
    90)
  set_tests_properties(
    test_instances
    PROPERTIES ENVIRONMENT
               "ROS_DOMAIN_ID=230;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()

ament_package()
