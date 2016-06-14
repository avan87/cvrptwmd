cmake_minimum_required(VERSION 3.5)
project(cvrptwmd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp  matr.cpp  utils.cpp cvrp_constants.cpp cvrp_types.cpp CVRPSolver.cpp CVRPSolver_server.cpp)

include_directories(/home/metis/Desktop/ortools/or-tools.Linux64/include)
include_directories(/home/metis/Downloads/or-tools-2016-04/src)

include_directories(/usr/local/include)

link_directories(/usr/local/lib)
link_directories(/home/metis/Desktop/ortools/or-tools.Linux64/lib)



add_executable(cvrptwmd ${SOURCE_FILES})

target_link_libraries(cvrptwmd ortools thrift thriftnb event )