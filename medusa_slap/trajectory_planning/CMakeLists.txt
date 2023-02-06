cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planning)

find_package(catkin 
# ROS components
REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 auv_msgs
 roscpp
 message_generation

  # Medusa gimmicks library is used for parameter loading
  farol_gimmicks_library
  medusa_slap_msg
)

find_package(
 # Eigen Library required
 Eigen3 REQUIRED
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 # Include for message generation
 message_runtime
)

add_compile_options(-std=c++17 -Wall) 

include_directories(
 include/trajectory_planning_ros
 include/trajectory_planning_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/trajectory_planning_ros/TrajectoryPlanningNode.cpp 
                                    src/trajectory_planning_ros/TrajectoryPlanningService.cpp
                                    src/trajectory_planning_algorithms/TrajectoryPlanningAlgorithm.cpp
                                    )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
