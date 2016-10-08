cmake_minimum_required(VERSION 2.6)
project(kondo_ics)

set(CMAKE_CXX_FLAGS "-Wall -Werror")

find_package(gflags)
if (NOT gflags_FOUND)
  add_definitions(-D gflags=google)
endif (NOT gflags_FOUND)

add_library(kondo_ics kondo_ics.cc)
target_link_libraries(kondo_ics gflags)

add_executable(kondo_ics_example kondo_ics_example.cc)
target_link_libraries(kondo_ics_example gflags kondo_ics)

add_executable(kondo_ics_tool kondo_ics_tool.cc)
target_link_libraries(kondo_ics_tool gflags kondo_ics)

add_executable(write_id write_id.cc)
target_link_libraries(write_id gflags kondo_ics)
