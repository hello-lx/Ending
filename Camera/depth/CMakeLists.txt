cmake_minimum_required( VERSION 3.0 )

project( show_depth )

set( CMAKE_CXX_STANDARD 11 )

set(OpenCV_DIR "/home/yuan/Xstudio/Opencv/opencv-4.2.0/build/")
find_package( OpenCV 4 REQUIRED )
include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
)
link_directories( 
    ${PROJECT_SOURCE_DIR}/OpenNI2/Drivers 
    ${PROJECT_SOURCE_DIR}
)

add_executable( show_depth preview.cpp )
target_link_libraries( show_depth ${OpenCV_LIBS} OpenNI2 )

add_executable( getCalibImgs getCalibImgs2.cpp )
target_link_libraries( getCalibImgs ${OpenCV_LIBS} OpenNI2 )

add_executable( getData getData.cpp )
target_link_libraries( getData ${OpenCV_LIBS} OpenNI2 )

add_executable( calib calib.cpp )
target_link_libraries( calib ${OpenCV_LIBS} )

add_executable( testCap testCap.cpp )
target_link_libraries( testCap ${OpenCV_LIBS} )
