# Project configuration
cmake_minimum_required(VERSION 2.8)
project(xpp_opt)

find_package(Eigen3 REQUIRED)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules") # to find Eigen
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/lib)
option(BUILD_SHARED_LIBS "Build shared libraries." ON)

set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the shared library")

# specify compiler flags. Default: Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug, Release(EIGEN_NO_DEBUG)." FORCE)
endif()


# The flags specified in this variable are added to every build.
#    -Wall" (show all warnings)
#    -std=c++0x  (add c++11 support for compilers gcc 4.3 or later)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
 # If Release, preprocessor macro "EIGEN_NO_DEBUG" for speedup
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D EIGEN_NO_DEBUG")


# Include directories
include_directories(${EIGEN_INCLUDE_DIRS})
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
            
set(MY_LIBS ${LIB_NAME}
            log4cxx)
           
                                      
# Some example executables if turned "ON"
option(BUILD_TESTS "Build some testing executables" ON) 
if(BUILD_TESTS)

add_executable(example 
    test/example.cc
    )
target_link_libraries(example ${MY_LIBS})

enable_testing()
find_package(GTest)  
if(GTEST_FOUND)
  set(MY_GTEST_UNITS  test/google/gtest_main.cc 
                      test/google/spliner_test.cc)
             
  add_executable(googleTest ${MY_GTEST_UNITS})
  
  target_link_libraries(googleTest 
                        ${MY_LIBS} 
                        ${GTEST_LIBRARIES} pthread) 
endif(GTEST_FOUND)
endif()

# To install and link to the library globally
# Install (ie copy) the header files
install(DIRECTORY include/ DESTINATION ${HEADER_INSTALL_ROOT} PATTERN *~ EXCLUDE)
# Install the shared library
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})

