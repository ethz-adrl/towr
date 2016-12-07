# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  kindr               # in hyq_spliner.cc
  xpp_common           
)

find_package(Eigen3 REQUIRED)
find_package(GTest)

# NLP solvers (adapt global path accordingly)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(IPOPT REQUIRED)
option(BUILD_SNOPT "Build problem to solve with snopt nlp solver" OFF)
if(BUILD_SNOPT)
  set(SNOPT_SRC_DIR "/home/winklera/3rd_party_software/snopt_lib")
  set(SNOPT_INCLUDE_DIRS ${SNOPT_SRC_DIR}/include)
  set(SNOPT_LIBS ${SNOPT_SRC_DIR}/lib/libsnopt7_cpp.so ${SNOPT_SRC_DIR}/lib/libsnopt7.so) # must be absolute path
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

set(LIB_NAME xpp_opt)

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
  INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS} ${SNOPT_INCLUDE_DIRS}
  LIBRARIES ${LIB_NAME} ${IPOPT_LIBRARIES} ${SNOPT_LIBS}
  CATKIN_DEPENDS xpp_common
  DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
  ${SNOPT_INCLUDE_DIRS}
)

set(SOURCES
  src/ellipse.cc
  src/qp_facade.cc
  src/zero_moment_point.cc
  src/linear_inverted_pendulum.cc
  src/motion_structure.cc
  src/a_robot_interface.cc
  # hyq
  src/step_sequence_planner.cc
  src/hyq_spliner.cc
  src/support_polygon.cc
  src/supp_polygon_container.cc
  src/hyq_robot_interface.cc
  # com
  src/com_motion.cc
  src/com_spline.cc
  src/com_spline6.cc    
  src/com_spline4.cc
  src/motion_factory.cc
  # nlp
  src/i_visualizer.cc
  src/motion_optimizer_facade.cc
  src/a_subject.cc
  src/variable_set.cc
  src/optimization_variables.cc
  src/i_observer.cc
  src/nlp_facade.cc
  src/nlp_observer.cc
  src/nlp.cc
  src/ipopt_adapter.cc
  # constraints
  src/a_constraint.cc
  src/cost_constraint_factory.cc
  src/a_linear_constraint.cc
  src/zmp_constraint_builder.cc
  src/linear_spline_equations.cc
  src/constraint_container.cc
  src/zmp_constraint.cc
  src/range_of_motion_constraint.cc
  src/obstacle_constraint.cc
  src/support_area_constraint.cc
  src/convexity_constraint.cc
  src/polygon_center_constraint.cc
  src/dynamic_constraint.cc
  # costs
  src/a_cost.cc
  src/a_spline_cost.cc
  src/cost_container.cc
  src/cost_function_functor.cc
  src/a_foothold_constraint.cc
  src/cost_adapter.cc
)

if(BUILD_SNOPT)
  set(SOURCES ${SOURCES} src/snopt_adapter.cc)
endif(BUILD_SNOPT)
             
# Add the shared library
add_library(${LIB_NAME}
   ${SOURCES}
)
target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${SNOPT_LIBS}
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
                         test/google/supp_polygon_test.cc
                         test/google/supp_polygon_container_test.cc
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
                         test/google/motion_structure_test.cc
#                        test/google/com_motion_test.cc
                         test/google/com_spline_test.cc
                         test/google/polynomial_test.cc
#                        test/google/spline_container_test.cc
#                        #constraints
#                        test/google/zmp_constraint_test.cc
#                        test/google/constraint_factory_test.cc
                         test/google/zmp_constraint_builder_test.cc
                         test/google/linear_spline_equations_test.cc
                         test/google/dynamic_constraint_test.cc
#                        test/google/joint_angles_constraint_test.cc
#                        test/google/constraint_container_test.cc
#                        test/google/range_of_motion_constraint_test.cc
#                        test/google/obstacle_constraint_test.cc
#                        #equations
#                        # costs
#                        test/google/cost_function_functor_test.cc
#                        test/google/cost_container_test.cc
#                        test/google/a_foothold_cost_test.cc
                         test/google/cost_adapter_test.cc
#                        # nlp optimizers
                         test/google/motion_optimizer_facade_test.cc
#                        test/google/nlp_test.cc
#                        test/google/snopt_test.cc
#                        test/google/snopt_adapter_test.cc
#                        test/google/ipopt_test.cc
#                        # general
#                        test/google/sparse_matrix_test.cc
                        )
               
    add_executable(googleTest ${MY_GTEST_UNITS})
    
    target_link_libraries(googleTest 
                          ${LIB_NAME}
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




