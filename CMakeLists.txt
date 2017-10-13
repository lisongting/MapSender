cmake_minimum_required(VERSION 2.8.3)
project(map_sender)

## Compile as C++11, supported in ROS Kinetic and newer
 #add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  openslam_gmapping
  roscpp
  rospy
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   MapStr.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_sender
  CATKIN_DEPENDS nav_msgs openslam_gmapping roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )




## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )




