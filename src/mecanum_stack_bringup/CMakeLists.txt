cmake_minimum_required(VERSION 3.8)
project(mecanum_stack_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/mecanum_kinematic_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_kinematic_sim
)

ament_package()
