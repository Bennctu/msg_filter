cmake_minimum_required(VERSION 3.0.2)
project(time_synchronizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  rospy
  image_transport
  cv_bridge
  genmsg
)

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_message_files(DIRECTORY /home/ncrl/My_WorkSpace/vs_code_ws/src/lvio/lvio_ros_msgs/msg FILES PointCloud3.msg 
                                                       CorrectData.msg IMUlow.msg PoseTwist.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_synchronizer
    src/synchor_node.cpp
)


target_link_libraries(vision_synchronizer ${catkin_LIBRARIES} ${OpenCV_LIBS}) 