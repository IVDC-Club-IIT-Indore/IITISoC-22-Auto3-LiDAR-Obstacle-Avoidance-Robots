cmake_minimum_required(VERSION 2.8.3)
project(hebi_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
)

## Mark files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

