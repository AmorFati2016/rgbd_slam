
add_library(preprocess imagepreprocess.cpp)
target_link_libraries(preprocess ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(main main.cpp imagepreprocess.cpp featureMatching.cpp)

target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})