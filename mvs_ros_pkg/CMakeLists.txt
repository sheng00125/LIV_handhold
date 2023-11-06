cmake_minimum_required(VERSION 3.0.2)
project(mvs_ros_pkg)
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake) 
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
)
find_package(OpenCV)
find_package(PCL REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
  lib
  /opt/MVS/lib/64
)

add_executable(grabImg src/grab.cpp)
add_executable(grabImgWithTrigger src/grab_trigger.cpp)
add_executable(Trigger_Image src/Trigger_Image.cpp)

target_link_libraries(
  grabImg
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
)
target_link_libraries(
  grabImgWithTrigger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
)

target_link_libraries(
  Trigger_Image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
)

add_executable(parse_type_time src/parse_type.cpp)
target_link_libraries(parse_type_time ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(parse_topic_time src/parse_topic.cpp)
target_link_libraries(parse_topic_time ${catkin_LIBRARIES} ${PCL_LIBRARIES})