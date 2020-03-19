cmake_minimum_required( VERSION 3.0 )

project( show_stereo )

set( CMAKE_CXX_STANDARD 11 )

find_package( PCL 1.9 REQUIRED )
# find_package( PCL 1.9 REQUIRED COMPONENTS common io visualization)
include_directories( ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message( "PCL_VERSION: " ${PCL_VERSION} )

add_executable( pcd_vis pcd.cpp )
target_link_libraries( pcd_vis ${PCL_LIBRARIES})

add_executable( ply_vis ply.cpp )
target_link_libraries( ply_vis ${PCL_LIBRARIES})

add_executable( test1 test1.cpp )
target_link_libraries( test1 
    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} 
    ${PCL_VISUALIZATION_LIBRARIES}
)
    
add_executable( test2 test2.cpp )
target_link_libraries( test2 
    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} 
    ${PCL_VISUALIZATION_LIBRARIES})
