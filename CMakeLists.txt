cmake_minimum_required(VERSION 2.8.3)
project(webots_manipulation)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  rospy
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
