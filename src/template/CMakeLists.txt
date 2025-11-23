cmake_minimum_required(VERSION 3.0.2)
project(template)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  image_transport
  laser_geometry
  tf
  mavros
  mavros_msgs
)

## 添加Eigen3依赖
find_package(Eigen3 REQUIRED)

## 添加PCL依赖（如果需要点云处理）
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(template src/template.cpp)
target_link_libraries(template ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
