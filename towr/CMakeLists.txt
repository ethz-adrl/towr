cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(towr VERSION 1.1.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(ifopt REQUIRED)


#############################################
## Generate catkin find scripts (optional) ##
#############################################
# These can then be used to "find_package(towr)"
find_package(catkin QUIET)
if (catkin_FOUND)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS ifopt
)
endif()


###########
## Build ##
###########
IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

# so dynamic library libtowr.so retains link to ifopt_core.so
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib) 

# The motion-planning problem formulated through variables and constraints
add_library(${PROJECT_NAME} SHARED
  src/towr.cc
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
target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    ${ifopt_LIBRARIES} # actually only needs ifopt_core --> solver independent 
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ifopt_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} 
  PUBLIC 
    cxx_range_for 
    cxx_auto_type
)


#############
## Testing ##
#############
enable_testing()
# example problem solved with IPOPT
add_executable(${PROJECT_NAME}-example 
  test/hopper_example.cc
)
target_link_libraries(${PROJECT_NAME}-example  
  PRIVATE
    ${PROJECT_NAME} 
)
add_test(${PROJECT_NAME}-example ${PROJECT_NAME}-example)

# unit tests of costs/constraints
find_package(GTest QUIET)
if (TARGET GTest::GTest) # only build when modern targets exists

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
include(GNUInstallDirs) # for correct libraries locations across platforms
set(config_package_location share/towr/cmake) # for .cmake find-scripts installs
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/towr/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/towr
)
# Auto generate and install the find scripts helpers
install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE towr::
  DESTINATION ${config_package_location}
)
# generate the file TowrConfigVersion.cmake
set(version_file "${CMAKE_CURRENT_BINARY_DIR}/towr-config-version.cmake")
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${version_file}
  VERSION ${towr_VERSION}
  COMPATIBILITY SameMajorVersion
)

# install the two files in a place where cmake looks for them so can call
# "find_package(towr).
# see also: https://cmake.org/cmake/help/v3.0/manual/cmake-packages.7.html
install(
  FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/towr-config.cmake"   # self-written
    ${version_file}                                         # generated
  DESTINATION ${config_package_location}
)