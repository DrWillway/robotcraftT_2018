cmake_minimum_required(VERSION 2.8.3)
project(robotcraft2018_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(maze_solver src/maze_solver.cpp)
target_link_libraries(maze_solver ${catkin_LIBRARIES})
#add_dependencies(maze_solver robotcraft2018_maze)

add_executable(maze_solver_real src/maze_solver_real.cpp)
target_link_libraries(maze_solver_real ${catkin_LIBRARIES})
add_dependencies(maze_solver_real robotcraft2018_maze)

#add_executable(robot_driver src/robot_driver.cpp)
#target_link_libraries(robot_driver ${catkin_LIBRARIES})
#add_dependencies(robot_driver robotcraft2018_maze)
