cmake_minimum_required(VERSION 3.5)
project(cvrptwmd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp  matr.cpp  utils.cpp  limits.h CVRPTW.cpp cvrptw_model_types.cpp cvrptw_model_types.cpp cvrptw_model_constants.cpp CVRPTW_server.cpp)
#CVRPTW_server.cpp cvrptw_model_constants.cpp cvrptw_model_types.cpp CVRPTW.cpp
include_directories(/home/metis/ort-tools-master/or-tools-master/src)
include_directories(/home/metis/ort-tools-master/or-tools-master/src/gen/)
include_directories(/home/metis/ort-tools-master/or-tools-master/dependencies/install/include)

#include_directories(/home/metis/Downloads/or-tools-2016-04/src)

include_directories(/usr/local/include)

link_directories(/usr/local/lib)
link_directories(/home/metis/ort-tools-master/or-tools-master/lib)



add_executable(cvrptwmd ${SOURCE_FILES})

target_link_libraries(cvrptwmd ortools thrift thriftnb event pthread )