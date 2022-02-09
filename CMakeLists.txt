cmake_minimum_required(VERSION 3.10)
project(path_planning)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(./RRT)

add_executable(path_planning main.cpp)
target_link_libraries(path_planning ${OpenCV_LIBS})