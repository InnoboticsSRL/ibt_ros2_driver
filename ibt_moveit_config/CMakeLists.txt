cmake_minimum_required(VERSION 3.22)
project(ibt_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})