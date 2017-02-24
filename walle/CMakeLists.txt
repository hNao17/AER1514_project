cmake_minimum_required(VERSION 2.8.3)
project(walle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab_exams
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES )
#add_service_files(DIRECTORY srv FILES )

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
## Declare a cpp executable

#talker
add_executable(talker_node src/ros_basics/talker_listener/talker.cpp)
target_link_libraries (talker_node ${catkin_LIBRARIES})
#add_dependencies(talker gaitech_doc_generate_message_cpp)

#map_navigation
add_executable(map_navigation_node src/turtlebot/navigation/map_navigation/map_navigation.cpp)
target_link_libraries(map_navigation_node ${catkin_LIBRARIES})

#nav_test1
add_executable(nav_test1 src/turtlebot/navigation/map_navigation/nav_test1.cpp)
target_link_libraries(nav_test1 ${catkin_LIBRARIES})

#nav_test2
add_executable(nav_test2 src/turtlebot/navigation/map_navigation/nav_test2.cpp)
target_link_libraries(nav_test2 ${catkin_LIBRARIES})

#nav_test3
add_executable(nav_test3 src/turtlebot/navigation/map_navigation/nav_test3.cpp)
target_link_libraries(nav_test3 ${catkin_LIBRARIES})

#nav_test4
add_executable(nav_test4 src/turtlebot/navigation/map_navigation/nav_test4.cpp)
target_link_libraries(nav_test4 ${catkin_LIBRARIES})

#nqr_nav_test1
add_executable(qr_nav_test1.cpp src/turtlebot/navigation/map_navigation/qr_nav_test1.cpp)
target_link_libraries(qr_nav_test1.cpp ${catkin_LIBRARIES})



