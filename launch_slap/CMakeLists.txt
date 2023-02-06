cmake_minimum_required(VERSION 2.8.3)
project(launch_slap)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 roscpp
)

catkin_package(
 CATKIN_DEPENDS
)

add_compile_options(-std=c++11) 

include_directories(
 ${catkin_INCLUDE_DIRS}
)

