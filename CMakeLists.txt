cmake_minimum_required(VERSION 3.20)
project(oled_display_node)


################################################
## Set C Options
################################################
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################
## Package Dependencies
################################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  genmsg
  message_generation
)

# find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


################################################
## Declare ROS messages, services and actions
################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DisplayOutput.msg"
  DEPENDENCIES std_msgs
)


################################################
## Build
################################################
# target_include_directories(my_library
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# target_link_libraries(oled_display_node PUBLIC
#   rcpcpp::hardware_interface
#   pluginlib::pluginlib
#   rclcpp::rclcpp
#   std_msgs::std_msgs
#   genmsg::genmsg
#   message_runtime::message_runtime
# )


## Declare a cpp executable
add_executable(oled_display_node src/oled_display_node.cpp)
add_dependencies(oled_display_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )


install(TARGETS oled_display_node
    DESTINATION lib/${PROJECT_NAME})


#############
## Testing ##
#############
