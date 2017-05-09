cmake_minimum_required(VERSION 3.1)

project(openZedAPI)

#install path, comment if you are using your own path
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

################################################################################
# Include CMake dependencies
################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(CheckCXXCompilerFlag)

if(EXISTS ${CMAKE_SOURCE_DIR}/LocalConfig.cmake)
    include(${CMAKE_SOURCE_DIR}/LocalConfig.cmake)
endif()

################################################################################
# Options
################################################################################
option(WITH_CUDA "WITH_CUDA" ON)
option(WITH_cudaArrayBM "WITH_cudaArrayBM" ON)
option(WITH_QT "WITH_QT means opencv was compiled with QT and hence needs QT" ON)

if(${WITH_CUDA} AND ${WITH_cudaArrayBM})
	set(WITH_cudaArrayBM ON)
elseif(NOT ${WITH_CUDA} AND ${WITH_cudaArrayBM})
	set(WITH_cudaArrayBM OFF)#
	message("********************************************************************")
	message("*****************CUDA must be ON to use cudaArrayBM*****************")
	message("********************************************************************")
endif()




################################################################################
# Find packages
################################################################################

if(${WITH_CUDA})
	find_package(CUDA REQUIRED)
endif()

find_package(OpenCV REQUIRED)

find_package(Doxygen)

if(${WITH_cudaArrayBM})
	find_package(Git REQUIRED)
	find_package(Glog REQUIRED)
	set(cudaArray_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/stereo/cudaArrayBM/cudaArray)

	if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cudaArray-prefix")
	find_package(Git REQUIRED)
	include(ExternalProject)
	ExternalProject_Add(cudaArray
	  GIT_REPOSITORY https://github.com/trueprice/cudaArray.git
	  SOURCE_DIR ${cudaArray_SOURCE_DIR}
	  CONFIGURE_COMMAND ""
	    BUILD_COMMAND ""
	    INSTALL_COMMAND ""
	    LOG_DOWNLOAD ON
	)
	endif()

	set(CUDA_ARRAY_INCLUDE_DIR ${cudaArray_SOURCE_DIR}/cudaArray)

endif()

if(${WITH_QT})
	find_package(Qt5Core REQUIRED)
endif()

if(DOXYGEN_FOUND) 
configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in ${CMAKE_SOURCE_DIR}/Doxyfile @ONLY)
add_custom_target(doc ALL COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM)
#install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
#install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/latex DESTINATION share/doc)
endif(DOXYGEN_FOUND)

################################################################################
# Compiler specific configuration
################################################################################

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} \
  -std=c++11 -O3 --expt-extended-lambda --expt-relaxed-constexpr -Wno-deprecated-gpu-targets")

set(CMAKE_CXX_STANDARD 11) # only cudaArrayBM needs c++11 support, otherwise openzedZPI is c++03 complaint
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
################################################################################
# Add sources
################################################################################

add_subdirectory(src)



