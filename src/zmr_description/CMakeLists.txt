cmake_minimum_required(VERSION 3.8)
project(zmr_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
