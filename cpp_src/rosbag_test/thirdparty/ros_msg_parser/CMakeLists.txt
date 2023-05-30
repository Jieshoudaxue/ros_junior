cmake_minimum_required(VERSION 2.8.3)
project(ros_msg_parser)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ros_msg_parser
   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
   src/ros_parser.cpp
 )

target_link_libraries(ros_msg_parser ${catkin_LIBRARIES})
