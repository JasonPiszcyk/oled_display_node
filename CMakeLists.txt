cmake_minimum_required(VERSION 3.20)
project(oled_display_node)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()


################################################
## Package Dependencies
################################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  genmsg
  message_generation
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


################################################
## Declare ROS messages, services and actions
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#  Was Default   add_message_files( FILES Message1.msg Message2.msg)
#  Example       add_message_files(DIRECTORY msg FILES Num.msg)
add_message_files(DIRECTORY msg FILES DisplayOutput.msg)

## Generate services in the 'srv' folder
#  Was Default   add_service_files( FILES Service1.srv Service2.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs )


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
