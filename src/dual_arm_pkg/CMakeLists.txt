cmake_minimum_required(VERSION 3.8)
project(dual_arm_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_until_collision.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_until_collision
)

ament_package()

