cmake_minimum_required(VERSION 2.8.3)
project(engagement_queue_tracking)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)