cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(towr_core)

find_package(ifopt REQUIRED)

#################################
## Generate cmake config files ##
#################################
# These can then be used to "find_package(towr)"
find_package(catkin QUIET)
if (catkin_FOUND)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt
)
endif()


###########
## Build ##
###########
# The motion-planning problem formulated through variables and constraints
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
  src/spline_acc_constraint.cc
  src/parameters.cc
  src/linear_constraint.cc
  src/soft_constraint.cc
)

# example problem solved with IPOPT
add_executable(${PROJECT_NAME}-example 
  test/example.cc
)


##########
## Link ##
##########
# ifopt built within pure modern cmake; everything easy
if(TARGET ifopt::ifopt_core)
  
  target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
      ifopt::ifopt_core
  )
  target_include_directories(${PROJECT_NAME} 
    PUBLIC
      include
  )
  target_link_libraries(${PROJECT_NAME}-example  
    PRIVATE 
      ${PROJECT_NAME}
      ifopt::ifopt_ipopt
  )
# ifopt built with catkin, so find script a bit different  
else()
  
  target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
      ${ifopt_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME} 
    PUBLIC
      include 
      ${ifopt_INCLUDE_DIRS}
  )
  target_compile_features(${PROJECT_NAME} 
    PUBLIC 
      cxx_range_for 
      cxx_auto_type
  )
  target_link_libraries(${PROJECT_NAME}-example  
    PRIVATE
      ${PROJECT_NAME} 
  ) 
endif()


#############
## Testing ##
#############
enable_testing()
add_test(${PROJECT_NAME}-example  ${PROJECT_NAME}-example)
find_package(GTest QUIET)
if (TARGET GTest::GTest) # only build when target exists

  add_executable(${PROJECT_NAME}-test
    test/dynamic_constraint_test.cc
    test/centroidal_dynamics_test.cc
  )
  target_link_libraries(${PROJECT_NAME}-test
    PRIVATE
      ${PROJECT_NAME}
      GTest::GTest GTest::Main
  )
  add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
endif()


#############
## Install ##
#############
# Mark library for installation
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark header files for installation
#install(
#  DIRECTORY include/towr/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/towr
#  FILES_MATCHING PATTERN "*.h"
#)
