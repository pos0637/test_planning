cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

project(test_planning)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable (${PROJECT_NAME} planning.cpp roi.cpp filter.cpp obb.cpp edge.cpp edge2d.cpp resampling.cpp reconstruct.cpp eulerAngle.cpp miscs.cpp python_bridge.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})