cmake_minimum_required(VERSION 2.8.3)
project(towr_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  xpp_msgs
  roscpp
  rosbag
  keyboard
  xpp_vis      # only for rviz_colors.h
  xpp_states
  towr
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++11") 




## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UserCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  xpp_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
     message_runtime
     std_msgs
     xpp_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/nlp_optimizer_node.cc
  src/user_interface.cc
  src/rviz_terrain_builder.cc
  src/gait_generator.cc
  src/monoped_gait_generator.cc
  src/biped_gait_generator.cc
  src/quadruped_gait_generator.cc
  # robot models
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


add_executable(nlp_optimizer_node src/exe/nlp_optimizer_node_exe.cc)
target_link_libraries(nlp_optimizer_node 
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

add_executable(rosbag_traj_combiner src/exe/rosbag_traj_combiner.cc)
target_link_libraries(rosbag_traj_combiner
  ${catkin_LIBRARIES}
)

add_executable(rosbag_geom_msg_extractor src/exe/rosbag_geom_msg_extractor.cc)
target_link_libraries(rosbag_geom_msg_extractor
  ${catkin_LIBRARIES}
)
                                                 

#add_executable(qp_optimizer_node 
#  src/executable/qp_optimizer_node_exe.cc
#)
#
#target_link_libraries(qp_optimizer_node 
#  ${LIB_NAME}
#  ${catkin_LIBRARIES}
#)  
#
#add_executable(sample_qp_caller 
#    src/executable/sample_qp_caller.cc
#)
#target_link_libraries(sample_qp_caller 
#  ${LIB_NAME}
#  ${catkin_LIBRARIES}
#)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(xpp_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(xpp_ros_node src/xpp_ros_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(xpp_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(xpp_ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} nlp_optimizer_node user_interface_node rosbag_traj_combiner
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
