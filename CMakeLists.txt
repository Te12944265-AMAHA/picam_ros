cmake_minimum_required(VERSION 3.0.2)
project(picam_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3 REQUIRED)
	
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/publish_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

