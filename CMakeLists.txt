cmake_minimum_required(VERSION 2.8.3)
project(activity_exploration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  roslib
  actionlib
  smach_ros
  human_trajectory
  region_observation
  vision_people_logging
  topological_navigation
  strands_exploration_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES activity_exploration
  CATKIN_DEPENDS human_trajectory region_observation roslib rospy topological_navigation smach smach_ros strands_exploration_msgs actionlib vision_people_logging
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/counting_manager.py
#   src/region_observation/observe.py
#   src/periodic_poisson_processes/people_poisson.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
