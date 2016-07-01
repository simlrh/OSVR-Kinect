cmake_minimum_required(VERSION 2.8.12)
project(KinectV2Plugin) # Change this line.

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
find_package(osvr REQUIRED)
find_package( KinectSDK2 REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories( ${KinectSDK2_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )

osvr_convert_json(je_nourish_kinectv2_json
    je_nourish_kinectv2.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv2_json.h")





include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")

osvr_add_plugin(NAME je_nourish_kinectv2
    CPP
    SOURCES
    je_nourish_kinectv2.cpp
	KinectDevice.cpp
	KinectDevice.h
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv2_json.h")

target_link_libraries( je_nourish_kinectv2 ${KinectSDK2_LIBRARIES} )