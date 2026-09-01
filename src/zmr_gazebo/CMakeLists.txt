cmake_minimum_required(VERSION 3.14)
project(zmr_gazebo)

find_package(ament_cmake REQUIRED)

# Install all our directories into the ROS share so launch files can find them
install(DIRECTORY
  launch
  worlds
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
