cmake_minimum_required(VERSION 3.5)
project(cvrptwmd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp  matr.cpp  utils.cpp CVRPTW.cpp cvrptw_model_constants.cpp cvrptw_model_types.cpp CVRPTW.cpp CVRPTW_server.cpp limits.h)

include_directories(/home/metis/Desktop/ortools/or-tools.Linux64/include)
include_directories(/home/metis/Downloads/or-tools-2016-04/src)

include_directories(/usr/local/include)

link_directories(/usr/local/lib)
link_directories(/home/metis/Desktop/ortools/or-tools.Linux64/lib)



add_executable(cvrptwmd ${SOURCE_FILES})

target_link_libraries(cvrptwmd ortools thrift thriftnb event )