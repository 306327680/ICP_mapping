cmake_minimum_required(VERSION 3.21)
project(ICP_mapping)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

ADD_LIBRARY(poseIO SHARED include/g2oIO/PoseGraphIO.h include/g2oIO/PoseGraphIO.cpp)
target_link_libraries(poseIO g2o_core g2o_types_slam3d g2o_stuff g2o_core)
include_directories(include/g2oIO)

ADD_LIBRARY(registration SHARED include/registration/registration.cpp
        include/registration/registration.h)
target_link_libraries(registration ${PCL_LIBRARIES})
include_directories(include/registration)

add_executable(ICP_mapping main.cpp include/tools/tools.cpp include/tools/tools.h include/mapping/mapping.cpp include/mapping/mapping.h)
target_link_libraries(ICP_mapping
        poseIO
        registration
        ${PCL_LIBRARIES}
        ${roscpp_LIBRARIES})
