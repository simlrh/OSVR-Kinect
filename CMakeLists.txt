cmake_minimum_required(VERSION 2.8.12)
project(KinectPlugin) # Change this line.

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
find_package(osvr REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( KinectSDK REQUIRED ) 
find_package( KinectSDK2 REQUIRED )

include_directories( ${KinectSDK_INCLUDE_DIRS} ${KinectSDK2_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

osvr_convert_json(je_nourish_kinectv1_json
    je_nourish_kinectv1.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv1_json.h")

osvr_convert_json(je_nourish_kinectv2_json
    je_nourish_kinectv2.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv2_json.h")

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")

osvr_add_plugin(NAME je_nourish_kinect
    CPP
    SOURCES
	stdafx.h
	je_nourish_kinect.cpp
	KinectMath.cpp
	KinectMath.h
	KinectV1Device.cpp
	KinectV1Device.h
	KinectV2Device.cpp
	KinectV2Device.h
	"${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv1_json.h"
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_kinectv2_json.h")