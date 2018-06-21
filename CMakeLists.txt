cmake_minimum_required(VERSION 2.8.0)
project(cpgen)

set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2")

set(SOURCES
  cpgen.cpp
  com_track.cpp
  leg_track.cpp
  interpolation.cpp
  plan_footprints.cpp
)

set(INCLUDES
  cpgen.h
  com_track.h
  leg_track.h
  eigen_types.h
  interpolation.h
  plan_footprints.h
)

# find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
# include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/include/eigen3)
add_library(cpgen SHARED ${SOURCES})

install(TARGETS cpgen LIBRARY DESTINATION lib)
install(FILES ${INCLUDES} DESTINATION include/cpgen)
