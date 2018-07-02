# towr-config.cmake
# ---------
#
# Locates the towr library and includes in cmake project.
# Copyright (c) 2018, Alexander W. Winkler. All rights reserved.
#
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following IMPORTED targets:
#   towr::towr  - variables and constraints for legged locomotion
#
#
# Example usage
# ^^^^^^^^^^^^^
#
#   find_package(towr REQUIRED)
#   add_executable(foo my_problem.cc)
#   target_link_libraries(foo PUBLIC towr::towr)
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project. 
# These can be neccessary when building with catkin and not modern cmake
#   towr_INCLUDE_DIRS    - path to public include (.h) files
#   towr_LIBRARIES       - path to all libraries
#
#=============================================================================
include(CMakeFindDependencyMacro)
find_dependency(ifopt)

# these are autogenerate by cmake
include("${CMAKE_CURRENT_LIST_DIR}/towr-targets.cmake")

get_target_property(towr_INCLUDE_DIRS towr::towr INTERFACE_INCLUDE_DIRECTORIES)
list(APPEND towr_INCLUDE_DIRS ${ifopt_INCLUDE_DIRS})

get_property(towr_LIBRARIES TARGET towr::towr PROPERTY LOCATION)
list(APPEND towr_LIBRARIES ${ifopt_LIBRARIES})