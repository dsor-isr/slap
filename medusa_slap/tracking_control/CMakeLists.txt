cmake_minimum_required(VERSION 2.8.3)
project(tracking_control)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 auv_msgs
 farol_msgs
 roscpp
 message_generation
 waypoint

 # Medusa gimmicks library 
 farol_gimmicks_library

 # dekf library
  medusa_slap_msg
)

find_package(
  # Eigen Library required
  Eigen3 REQUIRED  
)

catkin_package(
 CATKIN_DEPENDS
 roscpp

)


# add_compile_options(-std=c++11) 
add_compile_options(-std=c++17 -Wall) 


include_directories(
 include/tracking_control_ros
 include/tracking_control_algorithms
 ${catkin_INCLUDE_DIRS}
 # Eigen Library is required for this package
 ${EIGEN3_INCLUDE_DIR}
 )

add_executable(${PROJECT_NAME}_node src/tracking_control_ros/TrackingControlNode.cpp 
                                    src/tracking_control_ros/TrackingService.cpp
                                    src/tracking_control_algorithms/TrackingControlAlgorithm.cpp
                               )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
