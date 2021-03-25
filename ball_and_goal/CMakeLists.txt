cmake_minimum_required(VERSION 3.0.2)
project(ball_and_goal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bica
  image_transport
  cv_bridge
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  bica
  std_msgs
  kobuki_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
)

find_package(OpenCV)
include_directories(
  include
  fsm
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/FindBall.cpp
   #src/${PROJECT_NAME}/FindYellowGoal.cpp
   src/${PROJECT_NAME}/FindBlueGoal.cpp
   src/${PROJECT_NAME}/Turn.cpp
   src/${PROJECT_NAME}/BallAndGoalBica.cpp
  

   fsm/ball_and_goal_bica.cpp
 )


  # Hay que comprobar si el nombre de los nodos puestos en este CMakeList 
  # concuerdan con los puestos en bica. 


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(find_ball_node src/find_ball_node.cpp src/ball_and_goal/FindBall.cpp)
add_dependencies(find_ball_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_ball_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(turn_node src/turn_node.cpp src/ball_and_goal/Turn.cpp)
add_dependencies(turn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(find_yellow_goal_node src/find_yellow_goal_node.cpp src/ball_and_goal/FindYellowGoal.cpp)
add_dependencies(find_yellow_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_yellow_goal_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(find_blue_goal_node src/find_blue_goal_node.cpp src/ball_and_goal/FindBlueGoal.cpp)
add_dependencies(find_blue_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_blue_goal_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ball_and_goal_node src/ball_and_goal_node.cpp src/ball_and_goal/BallAndGoalBica.cpp)
add_dependencies(ball_and_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_and_goal_node ${PROJECT_NAME} ${catkin_LIBRARIES})


roslint_cpp(
 src/find_ball_node.cpp
)





