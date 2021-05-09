cmake_minimum_required(VERSION 3.0.2)
project(behavior_hospital)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  rospy
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
  darknet_ros_msgs

  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

catkin_package(
 
  CATKIN_DEPENDS roscpp actionlib move_base_msgs std_msgs kobuki_msgs sensor_msgs cv_bridge darknet_ros_msgs
  image_transport pcl_conversions pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(LINK_LIBS
  hospital_navigation
  ${catkin_LIBRARIES}
)

add_executable(behavior_hospital_node
  src/behavior_hospital_node.cpp
  src/behavior_hospital/darknet_detector.cpp
  src/behavior_hospital/rgbd_distance.cpp)
target_link_libraries(behavior_hospital_node ${catkin_LIBRARIES})


add_executable(node_rgbd_distance src/node_rgbd_distance.cpp)
target_link_libraries(node_rgbd_distance ${catkin_LIBRARIES})

add_executable(darknet_node src/darknet_node.cpp)
target_link_libraries(darknet_node ${catkin_LIBRARIES})


roslint_cpp(
  src/node_rgbd_distance.cpp
  src/darknet_node.cpp
)
