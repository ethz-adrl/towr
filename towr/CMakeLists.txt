cmake_minimum_required(VERSION 2.8)
project(towr)

find_package(catkin REQUIRED)
find_package(ifopt REQUIRED)    # variables/constraints are solver independent.
find_package(ifopt_ipopt QUIET) # builds solver only if this is installed.
find_package(ifopt_snopt QUIET) # builds solver only if this is installed.

add_compile_options(-std=c++11)


##############################################################################
## Generates cmake config files for this package, used when "find_package() ##
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_solver
  CATKIN_DEPENDS ifopt
)


###########
## Build ##
###########
include_directories(
  include
  ${ifopt_INCLUDE_DIRS}
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
  ${ifopt_LIBRARIES}
)


## build the actual solver using IPOPT and SNOPT
if(${ifopt_ipopt_FOUND} AND ${ifopt_snopt_FOUND})
include_directories(
  INCLUDE_DIRECTORIES
  ${ifopt_ipopt_INCLUDE_DIRS}
  ${ifopt_snopt_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_solver src/towr.cc)
target_link_libraries(${PROJECT_NAME}_solver
  ${PROJECT_NAME}
  ${ifopt_ipopt_LIBRARIES}
  ${ifopt_snopt_LIBRARIES}
)

endif()


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_solver
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
catkin_add_gtest(${PROJECT_NAME}-test
    test/gtest_main.cc
    test/dynamic_constraint_test.cc
    test/centroidal_dynamics_test.cc
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    pthread
    )
endif()

