cmake_minimum_required(VERSION 3.8)
project(kuka_kvp_hw_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(kvp_joint_state_sim_node src/kvp_joint_state_sim_node.cpp)
ament_target_dependencies(kvp_joint_state_sim_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  kvp_joint_state_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
