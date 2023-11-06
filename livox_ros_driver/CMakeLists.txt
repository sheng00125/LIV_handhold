# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)

set(PYTHON_EXECUTABLE /usr/bin/python3)
#---------------------------------------------------------------------------------------
# Start livox_ros_driver project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver version: ${LIVOX_ROS_DRIVER_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
	system
	thread
	chrono
	)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	message_generation
	rosbag
	pcl_ros
	)

## Find pcl lib
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
#   Message2.msg
	)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
	)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also n    eed
catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
	pcl_ros
	)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## make sure the livox_sdk_static library is installed
find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)

if((NOT LIVOX_SDK_LIBRARY) OR (NOT EXISTS ${LIVOX_SDK_LIBRARY}))
	# couldn't find the livox sdk library
	message("Coudn't find livox sdk library!")
	message("Download Livox-SDK from github and build&install it please!")
	message("Download Livox-SDK from github and build&install it please!")
	message("Download Livox-SDK from github and build&install it please!")

	message("git clone Livox-SDK from github temporarily, only for ROS distro jenkins build!")

	# clone livox sdk source code from github
	execute_process(COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
	message("Try to pull the livox sdk source code from github")
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	execute_process(COMMAND git clone https://github.com/Livox-SDK/Livox-SDK.git ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	execute_process(COMMAND cmake .. WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	execute_process(COMMAND make WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	include_directories(
		./
		${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/sdk_core/include
	)

	link_directories(
		./
		${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build/sdk_core
	)

else()
	message("find livox sdk library success")
endif()

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    ""
    )

#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
    )

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#---------------------------------------------------------------------------------------
# source file
#---------------------------------------------------------------------------------------
target_sources(${PROJECT_NAME}_node
	PRIVATE
	livox_ros_driver/lvx_file.cpp
	livox_ros_driver/ldq.cpp
	livox_ros_driver/lds.cpp
	livox_ros_driver/lds_lvx.cpp
	livox_ros_driver/lds_lidar.cpp
	livox_ros_driver/lds_hub.cpp
	livox_ros_driver/lddc.cpp
	livox_ros_driver/livox_ros_driver.cpp
	timesync/timesync.cpp
	timesync/user_uart/user_uart.cpp
	common/comm/comm_protocol.cpp
	common/comm/sdk_protocol.cpp
	common/comm/gps_protocol.cpp
	)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    common
    common/rapidjson
    comon/rapdidxml
    common/comm
    timesync
    timesync/user_uart
    livox_ros_driver
    )

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	livox_sdk_static.a
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${APR_LIBRARIES}
	)


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
