cmake_minimum_required(VERSION 2.8.3)
project(cat_common)

## Find catkin and any catkin packages
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_msgs
  std_srvs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

add_library(${PROJECT_NAME} src/cloud_common.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
