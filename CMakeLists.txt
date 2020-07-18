cmake_minimum_required(VERSION 2.8.3)
project(webots_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
