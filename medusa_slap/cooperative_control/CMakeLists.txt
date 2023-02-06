cmake_minimum_required(VERSION 2.8.3)
project(cooperative_control)

find_package(catkin 
REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 medusa_slap_msg
 roscpp

 message_generation

 # Medusa gimmicks library 
 farol_gimmicks_library

)

catkin_package(
 CATKIN_DEPENDS
)

add_compile_options(-std=c++11) 

include_directories(
 include/cooperative_control_ros
 include/cooperative_control_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/cooperative_control_ros/CooperativeControlNode.cpp 
                                    src/cooperative_control_ros/CooperativeControlService.cpp
                                    src/cooperative_control_algorithms/CooperativeControlAlgorithm.cpp
                                    )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
