cmake_minimum_required(VERSION 2.8)
project(towr_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(ifopt_core REQUIRED)


#################################
## Generate cmake config files ##
#################################
# These can then be used to "find_package(towr)"
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt_core
)


###########
## Build ##
###########
include_directories(
  include
  ${ifopt_core_INCLUDE_DIRS}
)

# The motion planning problem formulated through variables and constraints
# --> solver independent
add_library(${PROJECT_NAME}
  src/nlp_factory.cc
  src/time_discretization_constraint.cc
  src/base_motion_constraint.cc
  src/height_map.cc
  src/terrain_constraint.cc
  src/swing_constraint.cc
  src/force_constraint.cc
  src/total_duration_constraint.cc
  src/state.cc
  src/polynomial.cc
  src/nodes.cc
  src/phase_nodes.cc
  src/base_nodes.cc
  src/node_cost.cc
  src/spline.cc
  src/phase_spline.cc
  src/node_spline.cc
  src/nodes_observer.cc
  src/spline_holder.cc
  src/euler_converter.cc
  src/phase_durations.cc
  src/phase_durations_observer.cc
  src/dynamic_model.cc
  src/centroidal_model.cc
  src/dynamic_constraint.cc
  src/range_of_motion_constraint.cc
  src/parameters.cc
  src/linear_constraint.cc
  src/soft_constraint.cc
)
target_link_libraries(${PROJECT_NAME} 
  ${ifopt_core_LIBRARIES}
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
  DIRECTORY include/towr/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/towr
  FILES_MATCHING PATTERN "*.h"
)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
catkin_add_gtest(${PROJECT_NAME}-test
    test/gtest_main.cc
    test/dynamic_constraint_test.cc
    test/centroidal_dynamics_test.cc
)
target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    pthread
)
endif()

