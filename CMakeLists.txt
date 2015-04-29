cmake_minimum_required( VERSION 2.8 )
project( app )
find_package( OpenCV REQUIRED )

set( FLYCAP_INCLUDE "/usr/include/flycapture" )
include_directories( ${FLYCAP_INCLUDE} )

add_executable( app FlyCamera.cpp )
target_link_libraries( app ${OpenCV_LIBS} -lflycapture )
