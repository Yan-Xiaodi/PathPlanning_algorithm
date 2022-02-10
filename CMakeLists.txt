cmake_minimum_required(VERSION 3.10)
project(path_planning)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(./RRT ./RRT* ./common)
set(RRT main.cpp common/map.cpp RRT/rrt.cpp)
set(RRT_star main.cpp common/map.cpp RRT*/rrt_star.cpp RRT/rrt.cpp)

add_executable(path_planning ${RRT_star})
target_link_libraries(path_planning ${OpenCV_LIBS})
