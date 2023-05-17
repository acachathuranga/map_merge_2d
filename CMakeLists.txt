cmake_minimum_required(VERSION 3.0.2)
project(map_merge_2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  cv_bridge
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES map_merge_2d
 CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf2 tf2_ros cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/map_merger_node.cpp
                                    src/util/topic_name_utils.cpp
                                    src/util/tf_utils.cpp
                                    src/map_discovery/topic_discovery.cpp
                                    src/cv_core/cv_core.cpp
                                    src/submaps/submap.cpp
                                    src/submaps/submap_matcher.cpp
                                    src/submaps/submap_merger.cpp
                                    src/map_merger.cpp
                                    )
## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})