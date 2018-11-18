cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    gazebo_msgs
    visualization_msgs
    gazebo_msgs
    nav_msgs
    tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_example
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)


#add_executable(${PROJECT_NAME}_node src/turtlebot_example_node_lab2.cpp)
add_executable(mapping src/mapping.cpp)



## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#)
target_link_libraries(mapping
    ${catkin_LIBRARIES}
)

