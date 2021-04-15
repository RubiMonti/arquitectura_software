cmake_minimum_required(VERSION 3.0.2)
project(hospital_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
 
  CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)

set(LINK_LIBS
  hospital_navigation
  ${catkin_LIBRARIES}
)

add_executable(navigation src/navigation.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES})
