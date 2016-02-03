# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(catkin)
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
  LIBRARIES ${LIB_NAME}
  #CATKIN_DEPENDS roscpp
  DEPENDS log4cxx
)




# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)


set(SOURCES  src/zmp_optimizer.cc
             src/zmp_spline.cc
             src/spline_container.cc    
             src/spliner.cc
             src/spliner_3d.cc
             src/hyq_state.cc
             src/orientation.cc
             src/supp_triangle.cc
             src/hyq_spliner.cc
             )


# Add the shared library
set(LIB_NAME xpp_opt)
add_library(${LIB_NAME} 
            ${SOURCES})
target_link_libraries(${LIB_NAME} 
  log4cxx
)                        
                                      
# Some example executables if turned "ON"
option(BUILD_TESTS "Build some testing executables" ON) 
if(BUILD_TESTS)

add_executable(example 
    test/example.cc
)
target_link_libraries(example 
  ${LIB_NAME}
)

enable_testing()
find_package(GTest)  
if(GTEST_FOUND)
  set(MY_GTEST_UNITS  test/google/gtest_main.cc 
                      test/google/spliner_test.cc)
             
  add_executable(googleTest ${MY_GTEST_UNITS})
  
  target_link_libraries(googleTest 
                        ${LIB_NAME} 
                        ${GTEST_LIBRARIES} pthread) 
endif(GTEST_FOUND)
endif()

# To install and link to the library globally
# Install (ie copy) the header files
set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the shared library")

install(DIRECTORY include/ DESTINATION ${HEADER_INSTALL_ROOT} PATTERN *~ EXCLUDE)
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})

