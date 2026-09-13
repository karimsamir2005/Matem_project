cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

# Description-only package — no compilation. Just install the share assets so
# launch files and other packages can resolve package://arm_description/...
install(
  DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
