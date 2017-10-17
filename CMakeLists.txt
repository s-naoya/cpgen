cmake_minimum_required(VERSION 2.8.3)
project(cpgen)
# add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2")

set(SOURCES
cpgen.cpp
com_track.cpp
leg_track.cpp
)

find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
add_library(cpgen SHARED ${SOURCES})

install(TARGETS cpgen LIBRARY DESTINATION lib/cpgen)
install(FILES cpgen.h com_track.h leg_track.h eigen_types.h DESTINATION include/cpgen)
