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
  sensor_msgs
  cv_bridge
  image_transport
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

#nav_test1
#add_executable(nav_test1 src/scripts/nav_test1.cpp)
#target_link_libraries(nav_test1 ${catkin_LIBRARIES})

#nav_test2
#add_executable(nav_test2 src/scripts/nav_test2.cpp)
#target_link_libraries(nav_test2 ${catkin_LIBRARIES})

#nav_test3
#add_executable(nav_test3 src/scripts/nav_test3.cpp)
#target_link_libraries(nav_test3 ${catkin_LIBRARIES})

#nav_test4
#add_executable(nav_test4 src/scripts/nav_test4.cpp)
#target_link_libraries(nav_test4 ${catkin_LIBRARIES})

#qr_nav_test
#add_executable(qr_nav_test src/scripts/qr_nav_test.cpp)
#target_link_libraries(qr_nav_test ${catkin_LIBRARIES})

#qr_pose
#add_executable(qr_pose src/scripts/qr_pose.cpp)
#target_link_libraries(qr_pose ${catkin_LIBRARIES})

#add_executable(qr_nav_test_vs src/scripts/qr_nav_test_vs.cpp)
#target_link_libraries(qr_nav_test_vs ${catkin_LIBRARIES})

#qr_nav_test_rotatedLaptop
#add_executable(qr_nav_test_rotatedLaptop src/scripts/qr_nav_test_rotatedLaptop.cpp)
#target_link_libraries(qr_nav_test_rotatedLaptop ${catkin_LIBRARIES})

#cancelGoal_test
#add_executable(cancelGoal_test src/scripts/cancelGoal_test.cpp)
#target_link_libraries(cancelGoal_test ${catkin_LIBRARIES})

#autoDock_test
#add_executable(autoDock_test src/scripts/autoDock_test.cpp)
#target_link_libraries(autoDock_test ${catkin_LIBRARIES})

#autoDock_test
#add_executable(autoDock_nav_test src/scripts/autoDock_nav_test.cpp)
#target_link_libraries(autoDock_nav_test ${catkin_LIBRARIES})

#nav_test_withTimer
#add_executable(nav_test_withTimer src/scripts/nav_test_withTimer.cpp)
#target_link_libraries(nav_test_withTimer ${catkin_LIBRARIES})

#videoFilter
#add_executable(videoFilter src/scripts/videoFilter.cpp)
#target_link_libraries(videoFilter ${catkin_LIBRARIES})

#qr_nav_test_zbar
#add_executable(qr_nav_test_zbar src/scripts/qr_nav_test_zbar.cpp)
#target_link_libraries(qr_nav_test_zbar ${catkin_LIBRARIES})

#time_test.cpp
#add_executable(time_test src/scripts/time_test.cpp)
#target_link_libraries(time_test ${catkin_LIBRARIES})

#time_test_subscriber.cpp
#add_executable(time_test_subscriber src/scripts/time_test_subscriber.cpp)
#target_link_libraries(time_test_subscriber ${catkin_LIBRARIES})

#supervisor_linkedList_test.cpp
#add_executable(supervisor_linkedList_test src/scripts/supervisor_linkedList_test.cpp)
#target_link_libraries(supervisor_linkedList_test ${catkin_LIBRARIES})



