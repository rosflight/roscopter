cmake_minimum_required(VERSION 3.20...3.28)
project(roscopter LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosflight_compat REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Legacy yaml-cpp support for Ubuntu 22.04
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
  set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}"
  )
endif()

#################
### LIBRARIES ###
#################

# Param Manager (exported)
add_library(roscopter_param_manager src/param_manager/param_manager.cpp)
add_library(roscopter::param_manager ALIAS roscopter_param_manager)
set_target_properties(roscopter_param_manager PROPERTIES
  EXPORT_NAME param_manager
)
target_compile_features(roscopter_param_manager PUBLIC cxx_std_17)
target_compile_options(roscopter_param_manager PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(roscopter_param_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(roscopter_param_manager PUBLIC rclcpp::rclcpp)

# Estimation (exported)
set_source_files_properties(src/utils/geomag.c PROPERTIES
  LANGUAGE C
  COMPILE_OPTIONS "-Wno-overlength-strings"
)
add_library(roscopter_estimation
  src/ekf/estimator_ros.cpp
  src/ekf/estimator_ekf.cpp
  src/ekf/estimator_continuous_discrete.cpp
  src/utils/geomag.c
)
add_library(roscopter::estimation ALIAS roscopter_estimation)
target_compile_features(roscopter_estimation PUBLIC c_std_99 cxx_std_17)
target_compile_options(roscopter_estimation PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(roscopter_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(roscopter_estimation
  PUBLIC
    roscopter::param_manager
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    rosflight_msgs::rosflight_msgs
    roscopter_msgs::roscopter_msgs
    Eigen3::Eigen
  PRIVATE
    rosflight_compat::compat_headers # FIXME: legacy support for Humble
    # ament_index_cpp::ament_index_cpp # FIXME: use when Humble is dropped
)

###################
### EXECUTABLES ###
###################

# Estimator
add_executable(estimator src/ekf/estimator_node.cpp)
target_compile_features(estimator PRIVATE cxx_std_17)
target_compile_options(estimator PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(estimator PRIVATE roscopter::estimation)

# External attitude transcriber
add_executable(ext_att_transcriber src/ekf/external_attitude_transcriber.cpp)
target_compile_features(ext_att_transcriber PRIVATE cxx_std_17)
target_compile_options(ext_att_transcriber PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ext_att_transcriber PRIVATE
  rclcpp::rclcpp
  rosflight_msgs::rosflight_msgs
  roscopter_msgs::roscopter_msgs
)

# Controller
add_executable(controller
  src/controller/controller_ros.cpp
  src/controller/controller_state_machine.cpp
  src/controller/controller_cascading_pid.cpp
  src/controller/simple_pid.cpp
)
target_compile_features(controller PRIVATE cxx_std_17)
target_compile_options(controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(controller PRIVATE
  roscopter::param_manager
  rclcpp::rclcpp
  rosflight_msgs::rosflight_msgs
  roscopter_msgs::roscopter_msgs
  Eigen3::Eigen
)

# Trajectory Follower
add_executable(trajectory_follower
  src/navigation/trajectory_follower_ros.cpp
  src/navigation/trajectory_follower.cpp
  src/controller/simple_pid.cpp
)
target_compile_features(trajectory_follower PRIVATE cxx_std_17)
target_compile_options(trajectory_follower PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(trajectory_follower PRIVATE
  roscopter::param_manager
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosflight_msgs::rosflight_msgs
  roscopter_msgs::roscopter_msgs
  Eigen3::Eigen
)

# Path Manager
add_executable(path_manager
  src/navigation/path_manager_ros.cpp
  src/navigation/path_manager.cpp
)
target_compile_features(path_manager PRIVATE cxx_std_17)
target_compile_options(path_manager PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_manager PRIVATE
  roscopter::param_manager
  rclcpp::rclcpp
  std_srvs::std_srvs
  roscopter_msgs::roscopter_msgs
  Eigen3::Eigen
)

# Path Planner
add_executable(path_planner src/navigation/path_planner.cpp)
target_compile_features(path_planner PRIVATE cxx_std_17)
target_compile_options(path_planner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_planner PRIVATE
  roscopter::param_manager
  rosflight_compat::compat_headers
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosflight_msgs::rosflight_msgs
  roscopter_msgs::roscopter_msgs
  yaml-cpp::yaml-cpp
)

###############
### Testing ###
###############

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(estimator_test test/estimator_test.cpp)
  target_link_libraries(estimator_test
    roscopter::estimation
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
    rosflight_msgs::rosflight_msgs
    roscopter_msgs::roscopter_msgs
    Eigen3::Eigen
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###############
### Install ###
###############

# Headers for exported/public libraries
install(
  DIRECTORY
    include/param_manager
    include/ekf
  DESTINATION include/${PROJECT_NAME}
)

# Exported/public libraries
install(
  TARGETS
    roscopter_param_manager
    roscopter_estimation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# C++ nodes/executables
install(
  TARGETS
    estimator
    ext_att_transcriber
    controller
    trajectory_follower
    path_manager
    path_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

########################
### ROS Finalization ###
########################

# export custom libraries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# export public cmake package dependencies (not targets)
ament_export_dependencies(
  rclcpp
  rosflight_msgs
  roscopter_msgs
  sensor_msgs
  Eigen3
)

ament_package()
