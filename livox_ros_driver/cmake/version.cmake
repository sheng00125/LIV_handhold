#---------------------------------------------------------------------------------------
# Get livox_ros_driver version from include/livox_ros_driver.h
#---------------------------------------------------------------------------------------
file(READ "${CMAKE_CURRENT_LIST_DIR}/../livox_ros_driver/include/livox_ros_driver.h" LIVOX_ROS_DRIVER_VERSION_FILE)
string(REGEX MATCH "LIVOX_ROS_DRIVER_VER_MAJOR ([0-9]+)" _  "${LIVOX_ROS_DRIVER_VERSION_FILE}")
set(ver_major ${CMAKE_MATCH_1})

string(REGEX MATCH "LIVOX_ROS_DRIVER_VER_MINOR ([0-9]+)" _  "${LIVOX_ROS_DRIVER_VERSION_FILE}")
set(ver_minor ${CMAKE_MATCH_1})
string(REGEX MATCH "LIVOX_ROS_DRIVER_VER_PATCH ([0-9]+)" _  "${LIVOX_ROS_DRIVER_VERSION_FILE}")
set(ver_patch ${CMAKE_MATCH_1})

if (NOT DEFINED ver_major OR NOT DEFINED ver_minor OR NOT DEFINED ver_patch)
    message(FATAL_ERROR "Could not extract valid version from include/livox_ros_driver.h")
endif()
set (LIVOX_ROS_DRIVER_VERSION "${ver_major}.${ver_minor}.${ver_patch}")
