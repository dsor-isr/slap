cmake_minimum_required(VERSION 2.8.3)
project(range_simulation)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 medusa_slap_msg
 auv_msgs
 roscpp

  # Medusa gimmicks library is used for parameter loading
  farol_gimmicks_library
)

find_package(
 # Eigen Library required
 Eigen3 REQUIRED
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 message_runtime
)

add_compile_options(-std=c++17 -Wall) 

include_directories(
 include/range_simulation_ros
 include/range_simulation_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/range_simulation_ros/RangeSimulationNode.cpp src/range_simulation_algorithms/RangeSimulationAlgorithm.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
