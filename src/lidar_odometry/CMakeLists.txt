cmake_minimum_required(VERSION 2.8.3)
project(lidar_odometry)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(feature_extraction src/feature_extraction.cpp)
target_link_libraries(feature_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scan_to_scan_odometry src/scan_to_scan_odometry.cpp)
target_link_libraries(scan_to_scan_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(scan_to_map_odometry src/scan_to_map_odometry.cpp)
target_link_libraries(scan_to_map_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})



