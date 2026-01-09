cmake_minimum_required(VERSION 3.5)
project(panda_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()