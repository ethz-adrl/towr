cmake_minimum_required(VERSION 2.8.3)
project(towr_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  keyboard
  std_msgs
  xpp_msgs
  message_generation
  xpp_states
  towr
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++11") 

## Generate messages in the 'msg' folder
add_message_files( FILES TowrCommand.msg )
generate_messages( DEPENDENCIES std_msgs xpp_msgs )


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/towr_ros_interface.cc
  src/user_interface.cc
  src/rviz_terrain_builder.cc
  src/gait_generator.cc
  src/monoped_gait_generator.cc
  src/biped_gait_generator.cc
  src/quadruped_gait_generator.cc
  src/anymal_model.cc
  src/hyq_model.cc
  src/biped_model.cc
  src/monoped_model.cc
  src/height_map_examples.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
# so messages are compiled into header files before building library
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


add_executable(towr_ros_interface src/exe/towr_ros_interface_exe.cc)
target_link_libraries(towr_ros_interface 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(user_interface_node src/exe/user_interface_node.cc)
target_link_libraries(user_interface_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(rviz_terrain_node src/exe/rviz_terrain_node.cc)
target_link_libraries(rviz_terrain_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(goal_pose_publisher src/exe/goal_pose_publisher.cc)
target_link_libraries(goal_pose_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#add_executable(rosbag_traj_combiner src/exe/rosbag_traj_combiner.cc)
#target_link_libraries(rosbag_traj_combiner
#  ${catkin_LIBRARIES}
#)
#
#add_executable(rosbag_geom_msg_extractor src/exe/rosbag_geom_msg_extractor.cc)
#target_link_libraries(rosbag_geom_msg_extractor
#  ${catkin_LIBRARIES}
#)
                                                 

#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} towr_ros_interface 
                          user_interface_node
                          rviz_terrain_node
                          goal_pose_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_xpp_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
