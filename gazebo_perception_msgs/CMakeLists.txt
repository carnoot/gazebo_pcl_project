cmake_minimum_required(VERSION 2.8.3)
project(gazebo_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

  add_message_files(
    FILES
    Object.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

  generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_perception_msgs
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

