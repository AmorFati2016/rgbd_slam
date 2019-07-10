
add_executable(main main.cpp imagepreprocess.cpp)

target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})