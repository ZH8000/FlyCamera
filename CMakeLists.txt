cmake_minimum_required( VERSION 2.8 )
project( app )

find_package( OpenCV REQUIRED )
#find_package( GTK2 REQUIRED gtk )

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
#include_directories(${GTK2_INCLUDE_DIRS})
#link_directories(${GTK2_LIBRARY_DIRS})
#add_definitions(${GTK2_CFLAGS_OTHER})

set( FLYCAP_INCLUDE "/usr/include/flycapture" )
include_directories( ${FLYCAP_INCLUDE} )

#add_executable( app FlyCamera.cpp )
#add_executable( app AlexMatch.cpp )
#add_executable( app WebCamera.cpp webcam.cpp)
#add_executable( app LineAndCircle.cpp )
add_executable( app findContours.cpp)
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

target_link_libraries( app ${OpenCV_LIBS} -lflycapture tesseract )
#target_link_libraries( app ${OpenCV_LIBS} -lflycapture tesseract ${GTK2_LIBRARIES} )
