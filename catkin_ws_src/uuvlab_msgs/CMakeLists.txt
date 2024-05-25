cmake_minimum_required(VERSION 2.8.3)
project(uuvlab_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   sensor_msgs
   std_msgs
   geometry_msgs
   message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  DVL_Serial.msg
  DVLBeam.msg
  DVL.msg
  Imu.msg
  ThrusterCmd.msg
  ThrusterPWMs.msg
  EKF.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})



