cmake_minimum_required(VERSION 2.8.3)
project(pydisco)

find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   message_generation
)

catkin_package(
  INCLUDE_DIRS utils
  CATKIN_DEPENDS roscpp rospy message_runtime
)

