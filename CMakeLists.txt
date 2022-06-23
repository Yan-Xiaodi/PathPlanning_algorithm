cmake_minimum_required(VERSION 3.10)
project(path_planning)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(./include ./common)
set(RRT main.cpp common/map.cpp src/rrt_star.cpp src/rrt.cpp src/rrt_connect.cpp src/Grrt_connect.cpp)

add_executable(path_planning ${RRT})
target_link_libraries(path_planning -lpthread ${OpenCV_LIBS})

 