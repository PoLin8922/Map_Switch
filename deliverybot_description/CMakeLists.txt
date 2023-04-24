cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_description)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
  rviz/rviz.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(FILES
  urdf/dbot.gazebo
  urdf/dbot.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(
  DIRECTORY
    urdf/meshes
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(
  DIRECTORY
    urdf/materials/textures
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/materials
)
