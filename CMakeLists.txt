cmake_minimum_required(VERSION 2.8.3)
project(lm_pantilt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
  FILES
  PanTiltState.msg
)

add_service_files(
  FILES
  Start.srv
  Stop.srv
  Pan.srv
  Tilt.srv
  PanTilt.srv
  Reset.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package()