cmake_minimum_required(VERSION 2.8.3)
project(towr_examples)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(ifopt_ipopt REQUIRED)
find_package(towr_core REQUIRED)


catkin_package() # so "rosrun towr_examples example" works


###########
## Build ##
###########
include_directories(
  ${ifopt_ipopt_INCLUDE_DIRS}
  ${towr_core_INCLUDE_DIRS}
)

add_executable(towr_ipopt_example example.cc)
target_link_libraries(towr_ipopt_example 
  ${ifopt_ipopt_LIBRARIES}
  ${towr_core_LIBRARIES}
)


#############
## Install ##
#############
install(
  TARGETS towr_ipopt_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

