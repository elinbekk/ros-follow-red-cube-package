cmake_minimum_required(VERSION 3.0.2)
project(follow_red_cube_package)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  pcl_ros # for depth camera
  pcl_conversions
  tf
)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES follow_red_cube_package
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_listener src/scripts/image_listener.cpp)

target_link_libraries(image_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(follow_red_cube src/scripts/follow_red_cube.cpp)

target_link_libraries(follow_red_cube
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)