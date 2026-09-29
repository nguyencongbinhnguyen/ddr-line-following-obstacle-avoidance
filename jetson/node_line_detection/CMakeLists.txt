cmake_minimum_required(VERSION 2.8.3)
project(kinect_line_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

add_message_files(
  FILES
  LineData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/kinect_line_node_12_9.py
  #scripts/kinect_line_node_anh.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
