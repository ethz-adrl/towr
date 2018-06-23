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
#   towr::towr_core  - variables and constraints for legged locomotion
#
#
# Example usage
# ^^^^^^^^^^^^^
#
#   find_package(towr REQUIRED)
#   add_executable(foo my_problem.cc)
#   target_link_libraries(foo PUBLIC towr::towr_core)
#
#=============================================================================


# these are autogenerate by cmake
include("${CMAKE_CURRENT_LIST_DIR}/towr-targets.cmake")

