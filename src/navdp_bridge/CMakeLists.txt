cmake_minimum_required(VERSION 3.0.2)
project(navdp_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navdp_bridge
#  CATKIN_DEPENDS rospy nav_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/navdp_node.py
  scripts/mpc_controller.py
  scripts/pure_pursuit.py
  scripts/trajectory_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
