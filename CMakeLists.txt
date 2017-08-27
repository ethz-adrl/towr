# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
)

find_package(Eigen3 REQUIRED)
find_package(GTest)

# NLP solvers (adapt global path accordingly)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(IPOPT REQUIRED)
option(BUILD_SNOPT "Build problem to solve with snopt nlp solver" ON)
if(BUILD_SNOPT)
  set(SNOPT_SRC_DIR "/home/winklera/3rd_party_software/snopt_lib")
  set(SNOPT_INCLUDE_DIRS ${SNOPT_SRC_DIR}/include)
  set(SNOPT_LIBRARIES ${SNOPT_SRC_DIR}/lib/libsnopt7_cpp.so ${SNOPT_SRC_DIR}/lib/libsnopt7.so) # must be absolute path
endif(BUILD_SNOPT)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++14")


set(MOTION_OPT_LIB xpp_opt)
set(SOLVER_LIB xpp_solve)         # 
set(STATES_LIB xpp_states)   # defines common interface robot states 

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
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${STATES_LIB} ${MOTION_OPT_LIB} ${IPOPT_LIBRARIES}
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
  ${SNOPT_INCLUDE_DIRS}
)

## General library for robot state interfaces used by many packages
## TODO: Find way to make other packages use this library without needing
## to install IPOPT, SNOPT
add_library(${STATES_LIB}
  src/state.cc
  src/robot_state_cartesian.cc
)
target_link_libraries(${STATES_LIB}
  ${catkin_LIBRARIES}
)


# A nlp solving framework for Eigen, Ipopt, Snopt
# to compile this the third-party solver libraries must be installed
add_library(${SOLVER_LIB}
  src/nlp.cc
  src/ipopt_adapter.cc
  src/snopt_adapter.cc
  src/composite.cc
)
target_link_libraries(${SOLVER_LIB}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${SNOPT_LIBRARIES}
)


# the specific motion optimization problem for legged robots
# only packages that actually want to implement the optimization problem need this
add_library(${MOTION_OPT_LIB}
  src/motion_optimizer_facade.cc
  src/linear_constraint.cc
  src/spline_constraint.cc
  src/cost_constraint_factory.cc
  src/soft_constraint.cc
  src/time_discretization_constraint.cc
  src/terrain_constraint.cc
  src/height_map.cc
#  src/contact_constraints.cc
#  src/contact_timings.cc
  # body motion
  src/polynomial.cc
  src/node_values.cc
  src/node_cost.cc
  src/phase_nodes.cc
  src/spline.cc
  src/coeff_spline.cc
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
  src/dynamic_model.cc
#  src/lip_model.cc
  src/centroidal_model.cc
  src/dynamic_constraint.cc
  # kinematic
  src/range_of_motion_constraint.cc
#  src/polygon_center_constraint.cc
  #src/foothold_constraint.cc
  # robot specific info
  src/motion_parameters.cc
  src/motion_parameter_instances.cc  
)   
target_link_libraries(${MOTION_OPT_LIB}
  ${catkin_LIBRARIES}
  ${STATES_LIB}
  ${SOLVER_LIB}
)
  
# Some example executables if turned "ON"
option(BUILD_TESTS "Build some testing executables" ON)
if(BUILD_TESTS)
  
  # enable_testing()
  # FIXME use catkins macro to build the unit test
  # http://wiki.ros.org/catkin/CMakeLists.txt#msgs_srvs_actions
  # catkin_add_gtest(myUnitTest test/utest.cpp)
  message(STATUS "GTEST_FOUND " ${GTEST_FOUND})
  if(GTEST_FOUND)
    set(MY_GTEST_UNITS  test/google/gtest_main.cc 
                        test/google/helpers.cc
#                        test/google/lip_model_test.cc
#                        test/google/polynomial_test.cc
#                        test/google/ee_phase_motion_test.cc
#                        test/google/ee_motion_test.cc
                        test/dynamic_constraint_test.cc
#                        test/endeffectors_test.cc
                        test/centroidal_dynamics_test.cc
                        )
               
    add_executable(googleTest ${MY_GTEST_UNITS})
    
    target_link_libraries(googleTest 
                          ${SOLVER_LIB}
                          ${MOTION_OPT_LIB}
                          ${GTEST_LIBRARIES}
                          pthread
                          )
  endif(GTEST_FOUND)
endif()

