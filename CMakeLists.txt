# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

SET(IPOPT_SOURCE_DIR "/home/winklera/Software/Ipopt-3.12.4")

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
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${LIB_NAME}
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  DEPENDS log4cxx ${IPOPT_LIBS} 
)



# Include directories
SET(IPOPT_SOURCE_DIR "/home/winklera/Software/Ipopt-3.12.4") # these two lines must be included everywhere where this project is needed
link_directories(${IPOPT_SOURCE_DIR}/build/lib)
set(IPOPT_LIBS
    ipopt
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
 "${IPOPT_SOURCE_DIR}/build/include/coin"
)

set(SOURCES  
  src/geometric_structs.cc
  src/qp_optimizer.cc
  src/foothold.cc
  src/nlp_optimizer.cc
  src/zmp_spline.cc
  src/spline_container.cc    
  src/continuous_spline_container.cc    
  src/spliner.cc
  src/spliner_3d.cc
  src/hyq_state.cc
  src/orientation.cc
  src/support_polygon.cc
  src/supp_polygon_container.cc
  src/hyq_spliner.cc
  src/nlp_ipopt_zmp.cc
  src/zmp_constraint_builder.cc
  src/zero_moment_point.cc
  src/optimizer_node_base.cc
  src/nlp_optimizer_node.cc
  src/qp_optimizer_node.cc
  # observer pattern code
  src/a_subject.cc
  src/optimization_variables.cc
  src/i_observer.cc
  # constraints
  src/a_constraint.cc
  src/a_linear_constraint.cc
  src/initial_acceleration_equation.cc
  src/final_state_equation.cc
  src/spline_junction_equation.cc
  src/constraint_container.cc
  src/zmp_constraint.cc
  src/range_of_motion_constraint.cc
  # costs
  src/a_quadratic_cost.cc
  src/total_acceleration_equation.cc
  src/cost_container.cc
  src/foothold_nominal_deviation.cc
  src/range_of_motion_cost.cc
  # ros
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
                        test/google/spliner_test.cc
                        test/google/point2d_manipulations_test.cc
                        test/google/supp_polygon_test.cc
                        test/google/supp_polygon_container_test.cc
                        test/google/spline_container_test.cc
                        test/google/helpers.cc
                        test/google/zmp_constraint_builder_test.cc
                        test/google/zero_moment_point_test.cc
                        test/google/nlp_optimizer_test.cc
                        test/google/hyq_spliner_test.cc
                        #test/google/eigen_num_diff_test.cc
                        test/google/optimization_variables_test.cc
                        test/google/initial_acceleration_equation_test.cc
                        test/google/zmp_constraint_test.cc
                        test/google/constraint_container_test.cc
                        test/google/range_of_motion_constraint_test.cc
                        # costs
                        test/google/a_quadratic_cost_test.cc
                        test/google/total_acceleration_equation_test.cc
                        test/google/range_of_motion_cost_test.cc
                        test/google/cost_container_test.cc
                        test/google/foothold_nominal_deviation_test.cc
                        test/google/optimization_visualizer_test.cc
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




