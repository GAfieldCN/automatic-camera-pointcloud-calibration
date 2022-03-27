cmake_minimum_required(VERSION 2.8.3)
project(automatic-camera-pointcloud-calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

##set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
  rosbag
  cv_bridge
  tf
)

find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)
find_package(Ceres REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(GetExtrinsic src/get_extrinsic.cpp src/common.h)
target_link_libraries(GetExtrinsic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(getExt2 src/cam_lid_external2.cpp src/common.h)
target_link_libraries(getExt2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(projectCloud src/projectCloud.cpp src/common.h)
target_link_libraries(projectCloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(colorLidar src/color_lidar_display.cpp src/common.h)
target_link_libraries(colorLidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(feature_extraction_camera src/feature_extraction_camera.cpp src/common.h)
target_link_libraries(feature_extraction_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(feature_extraction_pointcloud src/feature_extraction_pointcloud.cpp src/common.h)
target_link_libraries(feature_extraction_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
