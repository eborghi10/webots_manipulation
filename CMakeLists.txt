cmake_minimum_required(VERSION 2.8.3)
project(webots_manipulation)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_grasps
    moveit_visual_tools
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
target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Demo grasp executable
add_executable(grasp_generator_demo src/grasp_generator_demo.cpp)
target_link_libraries(grasp_generator_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Demo grasp pipeline
add_executable(grasp_pipeline_demo src/grasp_pipeline_demo.cpp)
target_link_libraries(grasp_pipeline_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Move group interface demo
add_executable(move_group_interface_demo src/demo/move_group_interface.cpp)
target_link_libraries(move_group_interface_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Motion planning pipeline
add_executable(motion_planning_pipeline_demo src/demo/motion_planning_pipeline.cpp)
target_link_libraries(motion_planning_pipeline_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
    grasp_generator_demo
    grasp_pipeline_demo
    motion_planning_pipeline_demo
    move_group_interface_demo
    pick_and_place
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    launch
    urdf
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
