cmake_minimum_required(VERSION 2.8.3)
project(comms_slap)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 farol_msgs
 dekf
 rospy
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
)
