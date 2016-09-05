# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  #these should only be needed for the exe/ namespace
  xpp_controller   
  hyq_inverse_kinematics
)


# NLP solvers (adapt global path accordingly)
set(IPOPT_SRC_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.4")
set(IPOPT_INCLUDE_DIRS ${IPOPT_SRC_DIR}/build/include/coin)
set(IPOPT_LIBS         ${IPOPT_SRC_DIR}/build/lib/libipopt.so) # must be absolute path

SET(SNOPT_SRC_DIR "/home/winklera/3rd_party_software/snopt_lib")
set(SNOPT_INCLUDE_DIRS ${SNOPT_SRC_DIR}/include)
set(SNOPT_LIBS ${SNOPT_SRC_DIR}/lib/libsnopt7_cpp.so ${SNOPT_SRC_DIR}/lib/libsnopt7.so) # must be absolute path


find_package(Eigen3 REQUIRED)

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

# specify compiler flags. Default: Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug, Release(EIGEN_NO_DEBUG)." FORCE)
endif()



#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -fopenmp -O3")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG -fopenmp -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG")


set(LIB_NAME xpp_opt)


# Generate added messages with any dependencies listed here
add_message_files(
  FILES
  FootholdSequence.msg
  Foothold.msg
  Spline.msg
  StateLin3d.msg
#  SplineCoefficients.msg
  Footholds2d.msg
  RequiredInfoNlp.msg
  OptimizedParametersNlp.msg
  RequiredInfoQp.msg
  OptimizedParametersQp.msg
#  VariablesNlp.msg
)
add_service_files(
  FILES
#  SolveNlp.srv
  SolveQp.srv
  ReturnOptSplines.srv
  ReturnOptFootholds.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${IPOPT_INCLUDE_DIRS} ${SNOPT_INCLUDE_DIRS}
  LIBRARIES ${LIB_NAME} ${IPOPT_LIBS} ${SNOPT_LIBS}
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
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
  src/foothold.cc
  src/hyq_inverse_kinematics.cc
  src/spliner.cc
  src/spliner_3d.cc
  src/hyq_state.cc
  src/orientation.cc
  src/support_polygon.cc
  src/supp_polygon_container.cc
  src/hyq_spliner.cc
  src/nlp_facade.cc
  src/zmp_constraint_builder.cc
  src/zero_moment_point.cc
  src/motion_structure.cc
  src/a_inverse_kinematics.cc
  src/step_sequence_planner.cc
  src/line_equation.cc
  # com
  src/com_motion.cc
  src/com_spline.cc
  src/com_spline6.cc    
  src/com_spline4.cc    
  src/com_polynomial.cc
  src/motion_factory.cc
  #exe
  src/walking_controller.cc
  src/walking_controller_state.cc
  # nlp
  src/a_subject.cc
  src/nlp_structure.cc
  src/optimization_variables.cc
  src/optimization_variables_interpreter.cc
  src/i_observer.cc
  src/interpreting_observer.cc
  src/nlp.cc
  src/ipopt_adapter.cc
  src/snopt_adapter.cc
  # constraints
  src/a_constraint.cc
  src/cost_constraint_factory.cc
  src/a_linear_constraint.cc
  src/linear_spline_equations.cc
  src/constraint_container.cc
  src/zmp_constraint.cc
  src/range_of_motion_constraint.cc
#  src/joint_angles_constraint.cc
  src/obstacle_constraint.cc
  # costs
  src/a_spline_cost.cc
  src/cost_container.cc
  src/cost_function_functor.cc
  src/a_foothold_cost.cc
  # ros
  src/optimizer_node_base.cc
  src/nlp_optimizer_node.cc
  src/qp_optimizer_node.cc
  src/marker_array_builder.cc
  src/optimization_visualizer.cc
)
             
# Add the shared library
add_library(${LIB_NAME}
   ${SOURCES}
)
target_link_libraries(${LIB_NAME} 
  log4cxx
  ${catkin_LIBRARIES}
  ${IPOPT_LIBS}
  ${SNOPT_LIBS}
)  
# makes sure messages are built before the package
add_dependencies(${LIB_NAME} 
  ${LIB_NAME}_generate_messages_cpp
)

add_executable(nlp_optimizer_node 
  src/executable/nlp_optimizer_node_exe.cc
)
target_link_libraries(nlp_optimizer_node 
  ${LIB_NAME}
  ${catkin_LIBRARIES}
)   


add_executable(qp_optimizer_node 
  src/executable/qp_optimizer_node_exe.cc
)
target_link_libraries(qp_optimizer_node 
  ${LIB_NAME}
  ${catkin_LIBRARIES}
)  

add_executable(goal_state_pub 
  src/executable/goal_state_publisher.cc
)
target_link_libraries(goal_state_pub 
  ${LIB_NAME}
  ${catkin_LIBRARIES}
)                 
                                      
add_executable(sample_nlp_caller 
    src/executable/sample_nlp_caller.cc
)
target_link_libraries(sample_nlp_caller 
  ${LIB_NAME}
  ${catkin_LIBRARIES}
)

add_executable(sample_qp_caller 
    src/executable/sample_qp_caller.cc
)
target_link_libraries(sample_qp_caller 
  ${LIB_NAME}
  ${catkin_LIBRARIES}
)
  
# Some example executables if turned "ON"
option(BUILD_TESTS "Build some testing executables" ON) 
if(BUILD_TESTS)
  
  # enable_testing()
  # FIXME use catkins macro to build the unit test
  # http://wiki.ros.org/catkin/CMakeLists.txt#msgs_srvs_actions
  # catkin_add_gtest(myUnitTest test/utest.cpp)
  find_package(GTest)  
  message(STATUS "GTEST_FOUND " ${GTEST_FOUND})
  if(GTEST_FOUND)
    set(MY_GTEST_UNITS  test/google/gtest_main.cc 
#                        test/google/spliner_test.cc
                         test/google/point2d_manipulations_test.cc
                         test/google/supp_polygon_test.cc
                         test/google/supp_polygon_container_test.cc
                         test/google/helpers.cc
#                        test/google/zero_moment_point_test.cc
#                        test/google/nlp_facade_test.cc
##                        test/google/hyq_spliner_test.cc
#                        test/google/optimization_variables_test.cc
#                        test/google/interpreting_observer_test.cc
#                        test/google/optimization_visualizer_test.cc
#                        test/google/optimization_variables_interpreter_test.cc
#                        test/google/step_sequence_planner_test.cc
                         test/google/line_equation_test.cc
#                        # com
#                        test/google/com_motion_test.cc
                         test/google/com_spline_test.cc
#                        test/google/spline_container_test.cc
                         test/google/motion_factory_test.cc
#                        #constraints
#                        test/google/zmp_constraint_test.cc
#                        test/google/constraint_factory_test.cc
                         test/google/zmp_constraint_builder_test.cc
                         test/google/linear_spline_equations_test.cc
#                        test/google/joint_angles_constraint_test.cc
#                        test/google/constraint_container_test.cc
#                        test/google/range_of_motion_constraint_test.cc
#                        test/google/obstacle_constraint_test.cc
#                        #equations
#                        # costs
#                        test/google/cost_function_functor_test.cc
#                        test/google/cost_container_test.cc
#                        test/google/a_foothold_cost_test.cc
#                        # nlp optimizers
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
                          ${GTEST_LIBRARIES})
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




