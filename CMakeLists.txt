cmake_minimum_required(VERSION 3.0.2)
project(capra_sensor_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable( capra_sensor_map 
                src/sensor_mapping.cpp
                )

add_dependencies(capra_sensor_map
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(capra_sensor_map ${catkin_LIBRARIES})
