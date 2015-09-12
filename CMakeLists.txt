cmake_minimum_required(VERSION 2.8.3)
project(demo_lidar)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files


## Declare executables
add_executable(featureTracking src/featureTracking.cpp)
add_executable(visualOdometry src/visualOdometry.cpp)

add_executable(processDepthmap src/processDepthmap.cpp)

add_executable(transformMaintenance src/transformMaintenance.cpp)
add_executable(registerPointCloud src/registerPointCloud.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(featureTracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(visualOdometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(processDepthmap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(registerPointCloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
