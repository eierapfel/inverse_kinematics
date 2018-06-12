cmake_minimum_required(VERSION 2.8.3)
project(inverse_kinematics)

#Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  urdf
)


##Generate messages in the 'msg' folder

add_message_files(FILES destination_topic.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
)

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES inverse_kinematics
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(inverse_publisher src/inverse_publisher.cpp)
target_link_libraries(inverse_publisher ${catkin_LIBRARIES})

add_executable(inverse_subscriber src/inverse_subscriber.cpp)
target_link_libraries(inverse_subscriber ${catkin_LIBRARIES})

add_executable(joint_publisher1 src/joint_publisher1.cpp)
target_link_libraries(joint_publisher1 ${catkin_LIBRARIES})

add_executable(joint_publisher2 src/joint_publisher2.cpp)
target_link_libraries(joint_publisher2 ${catkin_LIBRARIES})

add_executable(inverse_joint_publisher src/inverse_joint_publisher.cpp)
target_link_libraries(inverse_joint_publisher ${catkin_LIBRARIES})
