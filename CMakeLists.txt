# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  xpp_states   
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


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")

set(MOTION_OPT_LIB xpp_opt)
set(SOLVER_LIB xpp_solve)

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
  LIBRARIES ${MOTION_OPT_LIB} ${IPOPT_LIBRARIES}
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

# A general nlp solving framework for Eigen, Ipopt, Snopt
add_library(${SOLVER_LIB}
#  src/optimization_variables_container.cc
#  src/optimization_variables.cc
  src/nlp.cc
  src/ipopt_adapter.cc
  src/snopt_adapter.cc
  src/composite.cc
  src/soft_constraint.cc
  src/time_discretization_constraint.cc
)
target_link_libraries(${SOLVER_LIB}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${SNOPT_LIBRARIES}
)

# the specific motion optimization problem for legged robots
add_library(${MOTION_OPT_LIB}
  src/linear_inverted_pendulum.cc
  src/motion_optimizer_facade.cc
  src/polynomial.cc
  src/polynomial_fifth_order.cc
  # base motion
  src/base_motion.cc
  src/com_polynomial_helpers.cc
  src/com_spline.cc
  src/com_spline6.cc    
  src/linear_spline_equations.cc
  # endeffector
  src/ee_phase_motion.cc
  src/ee_motion.cc
  src/endeffectors_motion.cc
  src/endeffector_load.cc
  src/contact_schedule.cc
  # constraints and costs
  src/linear_constraint.cc
  src/cost_constraint_factory.cc
  src/range_of_motion_constraint.cc
  src/polygon_center_constraint.cc
  src/dynamic_constraint.cc
  src/foothold_constraint.cc
  src/polynomial_cost.cc
#  src/ellipse.cc
#  src/com_spline4.cc
#  src/convexity_constraint.cc
#  src/contact_load_constraint.cc
#  src/obstacle_constraint.cc
  # robot specific info
  src/motion_parameters.cc
  src/quadruped_motion_parameters.cc  
)   
target_link_libraries(${MOTION_OPT_LIB}
  ${catkin_LIBRARIES}
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
                        test/google/linear_inverted_pendulum_test.cc
#                        test/google/polynomial_test.cc
                        test/google/motion_optimizer_facade_test.cc
                        test/google/ee_phase_motion_test.cc
                        test/google/ee_motion_test.cc
                        test/dynamic_constraint_test.cc
                        test/endeffectors_test.cc
                        test/contact_schedule_test.cc
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

