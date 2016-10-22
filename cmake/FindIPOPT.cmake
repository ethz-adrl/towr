# FindIPOPT
# ---------
#
# Try to locate the IPOPT library, by looking for the environmental variable
# IPOPT_DIR.
#
# Create the following variables::
#
#  IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
#  IPOPT_LIBRARIES    - Default library to link against to use IPOPT
#  IPOPT_FOUND        - If false, don't try to use IPOPT
# 
# Author: Alexander W. Winkler (winklera@ethz.ch)

#set(IPOPT_SRC_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.4")
if(NOT EXISTS $ENV{IPOPT_DIR})
  message(FATAL_ERROR "Environmental variable IPOPT_DIR not set. Set the environmental variable (export IPOPT_DIR=/path/to/ipopt/root)")
endif()

set(IPOPT_INCLUDE_DIRS $ENV{IPOPT_DIR}/build/include/coin)
set(IPOPT_LIBRARIES    $ENV{IPOPT_DIR}/build/lib/libipopt.so) # must be absolute path

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_INCLUDE_DIRS IPOPT_LIBRARIES)

mark_as_advanced(
  IPOPT_INCLUDE_DIRS
  IPOPT_LIBRARIES
)
      