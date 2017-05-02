################################################################################
# set variable names
################################################################################

set(CPU_STEREO_SOURCE 
	${CMAKE_CURRENT_SOURCE_DIR}/opencvstereo.cpp # add your files to this variable if you have additional cpu stereo code
	PARENT_SCOPE # this makes these variables available in parent directory
)


set(CUDA_OPENCV_STEREO_SOURCE#
# creating empty symbol	
)


set(CUDA_EXT_STEREO_SOURCE#
# creating empty symbol	
)



set(CUDA_EXT_STEREO_LIBRARIES# 
# creating empty symbol	
)


if(${WITH_CUDA})
	#adding opencv cuda code, which is used by default
	set(CUDA_OPENCV_STEREO_SOURCE ${CUDA_OPENCV_STEREO_SOURCE} 
	${CMAKE_CURRENT_SOURCE_DIR}/cudaopencvstereo.cpp
	PARENT_SCOPE)
endif()


set(OPENCV_CUDA_LIBRARIES
	opencv_cudastereo 
	opencv_cudaimgproc
	opencv_cudawarping
	PARENT_SCOPE
)


if(${WITH_cudaArrayBM})
	 include_directories(${CUDA_ARRAY_INCLUDE_DIR})#set in the cmakelist in src/

	cuda_add_library(cudaArrayBMLib SHARED cudaArrayBM/StereoMatcher.h cudaArrayBM/StereoMatcher.cu)

	add_dependencies(cudaArrayBMLib cudaArray)

 	set(CUDA_EXT_STEREO_LIBRARIES ${CUDA_EXT_STEREO_LIBRARIES} ${GLOG_LIBRARIES} cudaArrayBMLib PARENT_SCOPE)#
 
	set(CUDA_EXT_STEREO_SOURCE ${CUDA_EXT_STEREO_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR}/cudaArrayBM.cpp
PARENT_SCOPE)
 
	install(TARGETS cudaArrayBMLib DESTINATION lib/)
	
endif()


