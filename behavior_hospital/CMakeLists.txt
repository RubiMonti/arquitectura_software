cmake_minimum_required(VERSION 3.0.2)
project(behaviour_hospital)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  rospy
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs

  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

catkin_package(
 
  CATKIN_DEPENDS roscpp actionlib move_base_msgs std_msgs kobuki_msgs sensor_msgs cv_bridge
  image_transport pcl_conversions pcl_ros
)

include_directories(
  #include
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

add_executable(node_rgbd_distance src/node_rgbd_distance.cpp)
target_link_libraries(node_rgbd_distance ${catkin_LIBRARIES})

add_executable(Go_goal.cpp src/Behavior_hospital/Go_goal.cpp)
target_link_libraries(Go_goal ${catkin_LIBRARIES})

add_executable(Find_object.cpp src/Behavior_hospital/Find_object.cpp)
target_link_libraries(Find_object ${catkin_LIBRARIES})

add_executable(Behavior_hospital_node.cpp src/Behavior_hospital/Behavior_hospital_node.cpp)
target_link_libraries(Behavior_hospital_node ${catkin_LIBRARIES})

roslint_cpp(
  src/node_rgbd_distance.cpp
  src/Behavior_hospital/Go_goal.cpp
  src/Behavior_hospital/Find_object.cpp
  src/Behavior_hospital/Behavior_hospital_node.cpp
)
