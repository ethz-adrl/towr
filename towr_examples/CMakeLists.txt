cmake_minimum_required(VERSION 2.8.3)
project(towr_examples)

add_compile_options(-std=c++11)

find_package(ifopt_ipopt REQUIRED)
find_package(towr REQUIRED)

###########
## Build ##
###########
include_directories(
  ${ifopt_ipopt_INCLUDE_DIRS}
  ${towr_INCLUDE_DIRS}
)

add_executable(example example.cc)
target_link_libraries(example 
  ${ifopt_ipopt_LIBRARIES}
  ${towr_LIBRARIES}
)
