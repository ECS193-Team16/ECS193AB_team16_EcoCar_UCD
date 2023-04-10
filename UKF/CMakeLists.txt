cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (ukf_highway src/test/main.cpp src/ukf.cpp src/test/tools.cpp src/test/render/render.cpp)

add_executable (ukf_ioblock src/RTMaps/main.cpp src/ukf.cpp )

target_link_libraries (ukf_highway ${PCL_LIBRARIES})




