cmake_minimum_required(VERSION 2.8.6)
project(traffic_flows)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(traffic_flows
  src/graph.cpp
  src/optimizer.cpp)

target_link_libraries(traffic_flows)

add_executable(demo
  src/demo.cpp)

target_link_libraries(demo
  traffic_flows)