cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(point-cloud-visualization)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (simple_visualizer src/simple_visualizer.cpp)
target_link_libraries (simple_visualizer ${PCL_LIBRARIES})

add_executable (advance_visualizer src/advance_visualizer.cpp)
target_link_libraries (advance_visualizer ${PCL_LIBRARIES})

add_executable (pick_point src/pick_point.cpp)
target_link_libraries (pick_point ${PCL_LIBRARIES})
