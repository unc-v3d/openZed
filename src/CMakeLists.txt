


include_directories(
   
    ${CUDA_INCLUDE_DIRS}
)

set(LIBRARIES
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
)

set(OPENCV_CUDA_LIBRARIES
  #opencv_cudastereo
  opencv_calib3d
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_core
  opencv_features2d
  opencv_videoio
)
 

cuda_add_library(stereo_matching
  StereoMatcher.h
  StereoMatcher.cu
)

 
add_executable(zedCameraApp main.cpp zedCamera.cpp planedetection.cpp)
target_compile_definitions(zedCameraApp PRIVATE)#USE_GPU=1
target_link_libraries(zedCameraApp
  ${LIBRARIES}
  ${OPENCV_CUDA_LIBRARIES}
  stereo_matching
)
install(TARGETS zedCameraApp DESTINATION bin/)
