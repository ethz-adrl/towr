# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  xpp_states
  ifopt
)

find_package(Eigen3 REQUIRED)
#find_package(GTest)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++11")


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
  CATKIN_DEPENDS ifopt
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## General library for robot state interfaces used by many packages
## TODO: Find way to make other packages use this library without needing
## to install IPOPT, SNOPT
#add_library(${STATES_LIB}
#  src/state.cc
#  src/robot_state_cartesian.cc
#)
#target_link_libraries(${STATES_LIB}
#  ${catkin_LIBRARIES}
#)



# the specific motion optimization problem for legged robots
# only packages that actually want to implement the optimization problem need this
add_library(${PROJECT_NAME}
  src/towr.cc
  src/nlp_factory.cc
  src/time_discretization_constraint.cc
  src/base_motion_constraint.cc
  src/height_map.cc
  src/terrain_constraint.cc
  src/swing_constraint.cc
  src/force_constraint.cc
  src/total_duration_constraint.cc
  #
  src/gait_generator.cc
  src/monoped_gait_generator.cc
  src/biped_gait_generator.cc
  src/quadruped_gait_generator.cc
#  src/contact_constraints.cc
#  src/contact_timings.cc
  # body motion
  src/polynomial.cc
  src/node_values.cc
  src/node_cost.cc
  src/phase_nodes.cc
  src/spline.cc
#  src/spline.cc
  src/angular_state_converter.cc
#  src/base_motion.cc
  
  # these could go back in sometime
#  src/linear_spline_equations.cc
#  src/polynomial_cost.cc

  # endeffector motion
  #src/ee_phase_motion.cc
  #src/ee_motion.cc
  #src/endeffectors_motion.cc
#  src/endeffectors_force.cc
#  src/ee_force.cc
  src/contact_schedule.cc
  # system dynamics
  src/robot_model.cc
  src/kinematic_model.cc
  src/dynamic_model.cc
#  src/lip_model.cc
  src/centroidal_model.cc
  src/dynamic_constraint.cc
  # kinematic
  src/range_of_motion_constraint.cc
#  src/polygon_center_constraint.cc
  #src/foothold_constraint.cc
  # robot specific info
  src/optimization_parameters.cc
  # generic classes
  src/linear_constraint.cc
  src/soft_constraint.cc
)   
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS ${PROJECT_NAME}
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


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(${PROJECT_NAME}_test
    test/gtest_main.cc 
    test/dynamic_constraint_test.cc
    test/centroidal_dynamics_test.cc
)
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

