cmake_minimum_required(VERSION 3.1)

project(RealTimeStereo)

set (CMAKE_CXX_STANDARD 11)


################################################################################
# Include CMake dependencies
################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(CheckCXXCompilerFlag)

if(EXISTS ${CMAKE_SOURCE_DIR}/LocalConfig.cmake)
    include(${CMAKE_SOURCE_DIR}/LocalConfig.cmake)
endif()


################################################################################
# Find packages
################################################################################
set(cudaArray_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/cudaArray)

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
include_directories(
   
    ${CUDA_ARRAY_INCLUDE_DIR}
)


find_package(CUDA REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Doxygen)
find_package(Qt5Core)
if(DOXYGEN_FOUND) 
add_custom_target(doc ALL COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM)


install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
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


################################################################################
# Add sources
################################################################################

add_subdirectory(src)
#add_subdirectory(src/cudaArray)
#install path
#SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)


