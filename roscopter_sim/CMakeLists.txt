cmake_minimum_required(VERSION 3.20...3.28)
project(roscopter_sim)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

#################
## Executables ##
#################

add_executable(sim_state_transcriber src/state_transcription_node.cpp)
target_compile_features(sim_state_transcriber PRIVATE cxx_std_17)
target_compile_options(sim_state_transcriber PRIVATE
  -Wall -Wextra -Wpedantic
  -fopenmp # TODO: is this necessary?
)
target_link_libraries(sim_state_transcriber PRIVATE
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  roscopter_msgs::roscopter_msgs
  rosflight_msgs::rosflight_msgs
  Eigen3::Eigen
)

#############
## Install ##
#############

# C++ nodes/executables
install(
  TARGETS
    sim_state_transcriber
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
