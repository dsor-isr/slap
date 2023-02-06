cmake_minimum_required(VERSION 2.8.3)
project(medusa_slap_msg)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 farol_msgs
 roscpp
 message_generation
)

# Include here your custom messages
add_message_files(
  DIRECTORY msg
)

# Include here your custom service
add_service_files(
  DIRECTORY srv

) 
# Put here the message dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 message_runtime
)




add_compile_options(-std=c++11) 

include_directories(
 include/medusa_slap_msg_ros
 include/medusa_slap_msg_algorithms
 ${catkin_INCLUDE_DIRS}
)

