cmake_minimum_required(VERSION 3.8)
project(greenhouse_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config behavior_trees launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
