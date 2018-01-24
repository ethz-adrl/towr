cmake_minimum_required(VERSION 2.8)
project(towr)

find_package(catkin REQUIRED COMPONENTS 
  ifopt
  ifopt_ipopt
  ifopt_snopt
)

add_compile_options(-std=c++11)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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

