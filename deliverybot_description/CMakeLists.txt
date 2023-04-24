cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_description)

find_package(catkin REQUIRED )

install(FILES
  rviz/rviz.rviz
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/rviz
)

install(FILES
  urdf/dbot.gazebo
  urdf/dbot.xacro
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/urdf
)

# install(FILES
#   urdf/meshes/body.dae
#   urdf/meshes/castor.dae
#   urdf/meshes/door.dae
#   urdf/meshes/kinect.dae
#   urdf/meshes/left_wheel.dae
#   urdf/meshes/pusher.dae
#   urdf/meshes/right_wheel.dae
#   urdf/meshes/sensor_laser.dae
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/meshes
# )
install(
  DIRECTORY
    urdf/meshes
  DESTINATION
      ${CATKIN_PACKAGE_BIN_DESTINATION}/urdf/meshes
)

install(
  DIRECTORY
    urdf/materials/textures
  DESTINATION
      ${CATKIN_PACKAGE_BIN_DESTINATION}/urdf/materials/textures
)

# install(FILES
#   urdf/materials/textures/kinect.png
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/materials/textures
# )