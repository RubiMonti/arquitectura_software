cmake_minimum_required(VERSION 3.0.2)
project(hospital_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  rospy
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
 
  CATKIN_DEPENDS roscpp actionlib move_base_msgs std_msgs kobuki_msgs sensor_msgs
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS} 
)

set(LINK_LIBS
  hospital_navigation
  ${catkin_LIBRARIES}
)

add_executable(navigation_once src/navigation_once.cpp)
target_link_libraries(navigation_once ${catkin_LIBRARIES})

add_executable(navigation_kobuki src/navigation_kobuki.cpp)
target_link_libraries(navigation_kobuki ${catkin_LIBRARIES})

add_executable(publish_points src/publish_points.cpp)
target_link_libraries(publish_points ${catkin_LIBRARIES})

add_executable(navigation_secuencial src/navigation_secuencial.cpp)
target_link_libraries(navigation_secuencial ${catkin_LIBRARIES})

