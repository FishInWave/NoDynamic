cmake_minimum_required(VERSION 2.8.3)

project(NoDynamic)

add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    ndt_omp)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(keyframe src/keyframe.cpp)

add_executable (NoDynamic src/NoDynamic.cpp)
target_link_libraries (NoDynamic keyframe)
target_link_libraries (NoDynamic ${PCL_LIBRARIES})
target_link_libraries (NoDynamic ${catkin_LIBRARIES})
target_link_libraries (NoDynamic libyaml-cpp.so)
#target_link_libraries (NoDynamic /home/xyw/catkin_ws/devel/lib/libndt_omp.so)
