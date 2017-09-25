if(${WITH_QT})
################################################################################
# Add sources
################################################################################
 
include_directories(${CMAKE_SOURCE_DIR}/src)
set(OPENCV_LIBRARIES
  opencv_highgui
  opencv_imgproc
  opencv_core
)

file(GLOB ZED_CAMERA_RECORDER_SRC
    "*.cpp"
)
qt5_wrap_ui(UIS_HDRS
     recorder_main_window.ui
)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(zed_camera_recorder  ${ZED_CAMERA_RECORDER_SRC} ${UIS_HDRS})

target_link_libraries(zed_camera_recorder openzed Qt5::Core Qt5::Widgets)


################################################################################
# INSTALL
################################################################################

#added this line for people who havent installed the libraries in /usr/local/lib and dont want to update LD_LIBRARY_PATH
set_target_properties(zed_camera_recorder PROPERTIES INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS zed_camera_recorder DESTINATION bin/) 

endif()
