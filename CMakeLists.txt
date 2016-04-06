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



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG")


set(LIB_NAME xpp_opt)


# Generate added messages with any dependencies listed here
add_message_files(
  FILES
  FootholdSequence.msg
)
generate_messages(
  DEPENDENCIES
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
  CATKIN_DEPENDS message_runtime
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
  src/qp_optimizer.cc
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
  src/cost_function.cc
  src/constraints.cc
  src/zmp_constraint.cc
  src/zmp_publisher.cc
  src/ellipse.cc
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
                                      
# Some example executables if turned "ON"
option(BUILD_TESTS "Build some testing executables" ON) 
if(BUILD_TESTS)
  add_executable(example 
      test/example.cc
  )
  target_link_libraries(example 
    ${LIB_NAME}
    ${catkin_LIBRARIES}
  )
  
  # enable_testing()
  find_package(GTest)  
  message(STATUS "GTEST_FOUND " ${GTEST_FOUND})
  if(GTEST_FOUND)
    set(MY_GTEST_UNITS  test/google/gtest_main.cc 
                        test/google/spliner_test.cc
                        test/google/point2d_manipulations_test.cc
                        test/google/supp_polygon_test.cc
                        test/google/supp_polygon_container_test.cc
                        test/google/spline_container_test.cc
                        test/google/helpers.cc)
               
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




