cmake_minimum_required(VERSION 3.0.2)
project(ros_simple_pkg)
add_definitions(-std=c++14 -Wall -Wextra)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)

# add_message_files(
#   DIRECTORY
#     msg
#   FILES
# )

# add_service_files(
#   DIRECTORY
#     srv
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
# )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ros_simple_lib
  CATKIN_DEPENDS 
    roscpp
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ros_simple_lib
  src/ros_simple.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_simple_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(publisher_test src/ros_1_test/simple_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(publisher_test
  ${catkin_LIBRARIES}
  ros_simple_lib
)

## Declare a C++ executable
add_executable(subscriber_test src/ros_1_test/simple_subscriber.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(subscriber_test
  ${catkin_LIBRARIES}
  ros_simple_lib
)