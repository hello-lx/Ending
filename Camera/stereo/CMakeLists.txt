cmake_minimum_required( VERSION 3.0 )

project( show_stereo )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set( CMAKE_CXX_STANDARD 11 )

# set( OpenCV_DIR "/home/yuan/Xstudio/Opencv/opencv-4.2.0/build" )
# find_package( OpenCV 4.2.0 REQUIRED )
set( OpenCV_DIR "/home/yuan/Xstudio/Opencv/opencv-3.4.9/build" )
find_package( OpenCV 3 REQUIRED )

message( "=====> OpenCV_VERSION: " ${OpenCV_VERSION} )
include_directories( ${OpenCV_DIRS} )

# add_executable( stereo_show show_stereo.cpp )
# target_link_libraries( stereo_show ${OpenCV_LIBS} )

add_executable( stereo_depth src/my_stereo_depth.cpp )
target_link_libraries( stereo_depth ${OpenCV_LIBRARIES} )

# add_executable( get_imgs get_imgs.cpp )
# target_link_libraries( get_imgs ${OpenCV_LIBS} )

# add_executable( stereo_calib_and_retify stereo_calib_and_retify.cpp )
# target_link_libraries( stereo_calib_and_retify ${OpenCV_LIBS} )

add_executable( my_test "test/my_test.cpp" )
target_link_libraries( my_test ${OpenCV_LIBS} )

# add_executable( get_dataset "get_dataset.cpp" )
# target_link_libraries( get_dataset ${OpenCV_LIBS} )

# add_executable( show_dataset "show_dataset.cpp" )
# target_link_libraries( show_dataset ${OpenCV_LIBS} )

# add_executable( disparity "get_depth.cpp" )
# target_link_libraries( disparity ${OpenCV_LIBS} )

# add_executable( rectify_and_depth rectify_and_depth.cpp )
# target_link_libraries( rectify_and_depth ${OpenCV_LIBS} )

add_executable( get_depth src/get_depth.cpp )
target_link_libraries( get_depth ${OpenCV_LIBS} )
