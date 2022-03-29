cmake_minimum_required(VERSION 3.21)
project(ICP_mapping)

set(CMAKE_CXX_STANDARD 14)
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

add_executable(ICP_mapping main.cpp include/tools/tools.cpp include/tools/tools.h include/mapping/mapping.cpp include/mapping/mapping.h)
target_link_libraries(ICP_mapping
        ${PCL_LIBRARIES}
        ${roscpp_LIBRARIES})