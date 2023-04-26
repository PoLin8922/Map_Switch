cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_control)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
)

catkin_package(CATKIN_DEPENDS
	rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
    scripts/deliverybot_hw_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY
    config
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(
  DIRECTORY
    launch
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION}
)