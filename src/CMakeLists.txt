################################################################################
# Add sources
################################################################################
add_subdirectory(stereo)


#message("********************************************************************")
#message("CPU_STEREO_SOURCE is " ${CPU_STEREO_SOURCE})
#message("CUDA_OPENCV_STEREO_SOURCE is " ${CUDA_OPENCV_STEREO_SOURCE})
#message("CUDA_EXT_STEREO_SOURCE is " ${CUDA_EXT_STEREO_SOURCE})
#message("CUDA_EXT_STEREO_LIBRARIES is " ${CUDA_EXT_STEREO_LIBRARIES})
#message("OPENCV_CUDA_LIBRARIES is " ${OPENCV_CUDA_LIBRARIES})
#message("********************************************************************")
 



set(OPENCV_LIBRARIES
  opencv_calib3d
  opencv_highgui
  opencv_imgproc
  opencv_core
  opencv_videoio
)


if(${WITH_CUDA})
	include_directories( ${CUDA_INCLUDE_DIRS})
endif()



add_executable(zedCameraApp main.cpp ${CUDA_OPENCV_STEREO_SOURCE} ${CPU_STEREO_SOURCE} ${CUDA_EXT_STEREO_SOURCE} planedetection.cpp zedCamera.cpp)

# if using cuda turn on the flag to use gpu in code so that gpu sstereo function definitons are used
if(${WITH_CUDA})

	target_compile_definitions(zedCameraApp PRIVATE USE_GPU=1)
#else condition doesnt exist because there is nothing to add
endif()

target_link_libraries(zedCameraApp
  ${LIBRARIES}
  ${OPENCV_LIBRARIES}
  ${OPENCV_CUDA_LIBRARIES}
  ${CUDA_EXT_STEREO_LIBRARIES}
)

#create a library with all the code
add_library(openzed SHARED ${CUDA_OPENCV_STEREO_SOURCE} ${CPU_STEREO_SOURCE} ${CUDA_EXT_STEREO_SOURCE} planedetection.cpp zedCamera.cpp)

target_link_libraries(openzed  
  ${LIBRARIES}
  ${OPENCV_LIBRARIES}
  ${OPENCV_CUDA_LIBRARIES}
  ${CUDA_EXT_STEREO_LIBRARIES})


################################################################################
# INSTALL
################################################################################

#added this line for people who havent installed the libraries in /usr/local/lib and dont want to update LD_LIBRARY_PATH
set_target_properties(zedCameraApp PROPERTIES INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS zedCameraApp DESTINATION bin/) 
install(TARGETS openzed DESTINATION lib/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/zedCamera.h ${CMAKE_CURRENT_SOURCE_DIR}/planedetection.h ${CMAKE_CURRENT_SOURCE_DIR}/datatypes.h ${CMAKE_CURRENT_SOURCE_DIR}/confParser.h DESTINATION include/)

if(DOXYGEN_FOUND) 
install(DIRECTORY ${CMAKE_BINARY_DIR}/html DESTINATION share/doc)
endif(DOXYGEN_FOUND)

