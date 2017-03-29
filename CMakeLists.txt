# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  kindr               # in hyq_spliner.cc
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


option(BUILD_SHARED_LIBS "Build shared libraries." ON)
# specify compiler flags. Default: Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug, Release(EIGEN_NO_DEBUG)." FORCE)
endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -fopenmp -O3")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG -fopenmp -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11") 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG")

set(MOTION_OPT_LIB xpp_opt)

# Generate added messages with any dependencies listed here
#add_message_files(
#  FILES
#)
#add_service_files(
#  FILES
#  SolveQp.srv
#  GetStateLin3d.srv
#  ReturnOptSplines.srv
#  ReturnOptFootholds.srv
#)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  xpp_msgs      # Foothold.msg
#)

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
set(SOLVER_LIB xpp_solve)
add_library(${SOLVER_LIB}
  src/subject.cc
  src/variable_set.cc
  src/optimization_variables.cc
  src/observer.cc
  src/nlp.cc
  src/ipopt_adapter.cc
  src/snopt_adapter.cc
  src/constraint.cc
  src/constraint_container.cc
  src/cost.cc
  src/cost_container.cc
  src/cost_function_functor.cc
  src/soft_constraint.cc
  src/parametrization.cc
)
target_link_libraries(${SOLVER_LIB}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${SNOPT_LIBRARIES}
)

# the specific motion optimization problem for legged robots
add_library(${MOTION_OPT_LIB}
  src/ellipse.cc
#  src/zero_moment_point.cc
  src/linear_inverted_pendulum.cc
  src/line_equation.cc
  src/motion_optimizer_facade.cc
  src/nlp_facade.cc
  src/polynomial.cc
  src/polynomial_fifth_order.cc
  # com
  src/base_motion.cc
  src/com_polynomial_helpers.cc
  src/com_spline.cc
  src/com_spline6.cc    
  src/com_spline4.cc
  src/motion_factory.cc
  src/linear_spline_equations.cc
  # endeffector
  src/ee_swing_motion.cc
  src/ee_motion.cc
  src/endeffectors_motion.cc
  src/endeffector_load.cc
  # Input
  src/center_of_pressure.cc
  # constraints and costs
  src/linear_constraint.cc
  src/cost_constraint_factory.cc
  src/range_of_motion_constraint.cc
  src/support_area_constraint.cc
  src/convexity_constraint.cc
  src/polygon_center_constraint.cc
  src/dynamic_constraint.cc
  src/contact_load_constraint.cc
#  src/a_foothold_constraint.cc
#  src/obstacle_constraint.cc
  src/a_spline_cost.cc
  # robot specific info
  src/step_sequence_planner.cc
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
#                        test/google/spliner_test.cc
                         test/google/point2d_manipulations_test.cc
#                         test/google/supp_polygon_test.cc
#                         test/google/supp_polygon_container_test.cc
                         test/google/helpers.cc
                         test/google/linear_inverted_pendulum_test.cc
#                        test/google/zero_moment_point_test.cc
##                        test/google/hyq_spliner_test.cc
#                        test/google/optimization_variables_test.cc
#                        test/google/nlp_observer_test.cc
#                        test/google/optimization_visualizer_test.cc
#                        test/google/step_sequence_planner_test.cc
                         test/google/line_equation_test.cc
                         # hyq
#                        # com
#                         test/google/motion_structure_test.cc
#                        test/google/com_motion_test.cc
                         test/google/polynomial_test.cc
#                        test/google/spline_container_test.cc
#                        #constraints
#                        test/google/zmp_constraint_test.cc
#                        test/google/constraint_factory_test.cc
#                         test/google/zmp_constraint_builder_test.cc
#                         test/google/linear_spline_equations_test.cc
#                        test/google/joint_angles_constraint_test.cc
#                        test/google/constraint_container_test.cc
#                        test/google/range_of_motion_constraint_test.cc
#                        test/google/obstacle_constraint_test.cc
#                        #equations
#                        # costs
#                        test/google/cost_function_functor_test.cc
#                        test/google/cost_container_test.cc
#                        test/google/a_foothold_cost_test.cc
#                         test/google/soft_constraint_test.cc
#                        # nlp optimizers
                         test/google/motion_optimizer_facade_test.cc
#                        test/google/nlp_test.cc
#                        test/google/snopt_test.cc
#                        test/google/snopt_adapter_test.cc
#                        test/google/ipopt_test.cc
#                        # general
#                        test/google/sparse_matrix_test.cc
                         test/google/ee_swing_motion_test.cc
                         test/google/ee_motion_test.cc
                         ## these should be kept well maintained
                         test/dynamic_constraint_test.cc
                         test/center_of_pressure_test.cc
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

# To install and link to the library globally
# Install (ie copy) the header files
#set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
#set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the shared library")
#
#install(
#  DIRECTORY include/ 
#  DESTINATION ${HEADER_INSTALL_ROOT} 
#  PATTERN *~ EXCLUDE
#  )
#install(
#  TARGETS ${LIB_NAME}
#  ARCHIVE DESTINATION ${LIB_INSTALL_ROOT}
#  LIBRARY DESTINATION ${LIB_INSTALL_ROOT}
#)




