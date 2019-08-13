cmake_minimum_required(VERSION 2.8)

project(rgbd_slam)

message("-- add define c++ 11")
add_definitions(-std=c++11)

message("-- SET EXECUTABLE_OUTPUT_PATH  " ${PROJECT_SOURCE_DIR} "/bin/")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

message("-- SET LIBRARY_OUTPUT_PATH  " ${PROJECT_SOURCE_DIR} "/lib/")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/lib)

message("-- FIND OPENCV LIBRARY")
find_package(OpenCV REQUIRED)
link_libraries(${OpenCV_LIBRARIES})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen REQUIRED)

message("-- FIND PCL LIBRARY")
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)