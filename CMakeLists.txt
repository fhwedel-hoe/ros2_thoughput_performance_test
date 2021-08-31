cmake_minimum_required(VERSION 3.5)
project(throughput_performance_test)

set (CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

set(NODE_NAME "Talker")
add_library(${NODE_NAME} SHARED src/talker.cpp)
ament_target_dependencies(${NODE_NAME} 
    rclcpp
    rclcpp_components
    image_transport
    camera_calibration_parsers
)
rclcpp_components_register_node(${NODE_NAME} PLUGIN "${PROJECT_NAME}::${NODE_NAME}" EXECUTABLE "talker_node")

set(NODE_NAME "Listener")
add_library(${NODE_NAME} SHARED src/listener.cpp)
ament_target_dependencies(${NODE_NAME} 
    rclcpp
    rclcpp_components
    image_transport
)
rclcpp_components_register_node(${NODE_NAME} PLUGIN "${PROJECT_NAME}::${NODE_NAME}" EXECUTABLE "listener_node")

# install nodes
install(TARGETS 
  Talker
  Listener
  DESTINATION lib
)

# install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
