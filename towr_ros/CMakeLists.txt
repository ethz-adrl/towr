cmake_minimum_required(VERSION 2.8.3)
project(towr_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  xpp_msgs
  message_generation
  xpp_states
  towr
)

find_package(Curses REQUIRED)


## Generate messages in the 'msg' folder
add_message_files( FILES TowrCommand.msg )
generate_messages( DEPENDENCIES std_msgs xpp_msgs )

catkin_package()


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## The main executable that runs TOWR
add_executable(towr_ros 
  src/towr_ros.cc
)
add_dependencies(towr_ros ${PROJECT_NAME}_gencpp) # build .msg first
target_link_libraries(towr_ros
  ${catkin_LIBRARIES}
)

## Keyboard interfaces and other helpers
add_executable(towr_user_interface src/towr_user_interface.cc)
add_dependencies(towr_user_interface ${PROJECT_NAME}_gencpp) # build .msg first
target_link_libraries(towr_user_interface 
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(rviz_terrain_publisher src/rviz_terrain_publisher.cc
  src/rviz_terrain_builder.cc
)
add_dependencies(rviz_terrain_publisher ${PROJECT_NAME}_gencpp) # build .msg first
target_link_libraries(rviz_terrain_publisher
  ${catkin_LIBRARIES}
)

add_executable(goal_pose_publisher src/goal_pose_publisher.cc)
add_dependencies(goal_pose_publisher ${PROJECT_NAME}_gencpp) # build .msg first
target_link_libraries(goal_pose_publisher
  ${catkin_LIBRARIES}
)

add_executable(rosbag_traj_combiner src/rosbag_traj_combiner.cc)
target_link_libraries(rosbag_traj_combiner
  ${catkin_LIBRARIES}
)

add_executable(rosbag_geom_msg_extractor src/rosbag_geom_msg_extractor.cc)
target_link_libraries(rosbag_geom_msg_extractor
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS towr_ros 
          towr_user_interface
          rviz_terrain_publisher
          goal_pose_publisher
          rosbag_traj_combiner
          rosbag_geom_msg_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
