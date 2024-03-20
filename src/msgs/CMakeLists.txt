cmake_minimum_required(VERSION 3.0.2)
project(robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GenericMsg.msg
  KeyValue.msg
)

add_service_files(
  FILES
  GenericSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)