cmake_minimum_required(VERSION 3.5)
project(ros_simple)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Define Include
include_directories(include)

# Add Library
add_library(ros_simple_lib SHARED
  src/ros_simple.cpp
)

set_target_properties(ros_simple_lib PROPERTIES VERSION "${$PROJECT_NAME}_VERSION}")
ament_target_dependencies(ros_simple_lib rclcpp)

install(
  TARGETS ros_simple_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


# Executables
add_executable(publisher_test src/ros_2_test/simple_publisher.cpp)
ament_target_dependencies(publisher_test rclcpp std_srvs)
target_link_libraries(publisher_test ros_simple_lib)

install(
  TARGETS publisher_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(subscriber_test src/ros_2_test/simple_subscriber.cpp)
ament_target_dependencies(subscriber_test rclcpp std_srvs)
target_link_libraries(subscriber_test ros_simple_lib)

install(
  TARGETS subscriber_test
  DESTINATION lib/${PROJECT_NAME}
)

# Launchers
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()