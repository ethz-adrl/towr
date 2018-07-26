cmake_minimum_required(VERSION 2.8.3)
project(towr_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  visualization_msgs
  message_generation
  xpp_msgs
  xpp_states
  towr
)


## Generate messages in the 'msg' folder
add_message_files( FILES TowrCommand.msg )
generate_messages( DEPENDENCIES std_msgs xpp_msgs )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS xpp_states roscpp
   DEPENDS towr
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# The interface to derive from for ros integration
add_library(${PROJECT_NAME} SHARED
  src/towr_ros_interface.cc
)
add_dependencies(${PROJECT_NAME} 
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## An example executable that runs TOWR
add_executable(towr_ros_app 
  src/towr_ros_app.cc
)
add_dependencies(towr_ros_app 
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(towr_ros_app
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


## Keyboard interfaces (ncurses)
find_package(Curses REQUIRED)
add_executable(towr_user_interface 
  src/towr_user_interface.cc
)
add_dependencies(towr_user_interface 
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(towr_user_interface 
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)


# Display different terrains in rviz
add_executable(rviz_terrain_publisher 
  src/rviz_terrain_publisher.cc
)
add_dependencies(rviz_terrain_publisher 
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(rviz_terrain_publisher
  ${catkin_LIBRARIES}
)


# Display goal pose in rviz
add_executable(goal_pose_publisher 
  src/goal_pose_publisher.cc
)
add_dependencies(goal_pose_publisher 
  ${PROJECT_NAME}_gencpp
) 
target_link_libraries(goal_pose_publisher
  ${catkin_LIBRARIES}
)


# (Optional) to manipulate ROS bags
add_executable(rosbag_traj_combiner 
  src/rosbag_traj_combiner.cc
)
target_link_libraries(rosbag_traj_combiner
  ${catkin_LIBRARIES}
)


add_executable(rosbag_geom_msg_extractor 
  src/rosbag_geom_msg_extractor.cc
)
target_link_libraries(rosbag_geom_msg_extractor
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS towr_ros_app
          towr_user_interface
          rviz_terrain_publisher
          goal_pose_publisher
          rosbag_traj_combiner
          rosbag_geom_msg_extractor
          ${PROJECT_NAME}
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
  DIRECTORY launch rviz bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
