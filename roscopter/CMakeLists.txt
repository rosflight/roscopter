set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.8)
project(roscopter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_dependencies(rclcpp rclpy roscopter_msgs rosflight_msgs std_msgs sensor_msgs Eigen3 geometry_msgs)

add_definitions(-DROSCOPTER_DIR="${CMAKE_CURRENT_LIST_DIR}")

install(
  DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} 
  ${ament_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  )


### LIBRARIES ###

# Param Manager
add_library(param_manager
  include/param_manager/param_manager.hpp
  src/param_manager/param_manager.cpp
)
ament_target_dependencies(param_manager rclcpp)
ament_export_targets(param_manager HAS_LIBRARY_TARGET)
install(DIRECTORY include/param_manager DESTINATION include)
install(TARGETS param_manager
  EXPORT param_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ESTIMATOR LIB
set_source_files_properties(geomag.c PROPERTIES LANGUAGE C)
add_library(estimation_lib SHARED
  src/ekf/estimator_ros.cpp
  src/ekf/estimator_ekf.cpp
  src/ekf/estimator_continuous_discrete.cpp
  src/utils/geomag.c)
target_include_directories(estimation_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ekf>"
    "$<INSTALL_INTERFACE:include/ekf>")
ament_target_dependencies(estimation_lib rclcpp geometry_msgs rosflight_msgs sensor_msgs Eigen3 roscopter_msgs ament_index_cpp)
install(DIRECTORY include/ekf DESTINATION include)
install(TARGETS estimation_lib
  EXPORT export_estimation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_estimation_lib HAS_LIBRARY_TARGET)

### EXECUTABLES ###

# Estimator
add_executable(estimator
              src/ekf/estimator_node.cpp)
ament_target_dependencies(estimator PUBLIC roscopter_msgs rosflight_msgs sensor_msgs rclcpp Eigen3 ament_index_cpp)
target_link_libraries(estimator PUBLIC estimation_lib ${ament_LIBRARIES} param_manager ${YAML_CPP_LIBRARIES})
target_include_directories(estimator PUBLIC include)
install(TARGETS
  estimator
  DESTINATION lib/${PROJECT_NAME})

# External attitude transcriber
add_executable(ext_att_transcriber
  src/ekf/external_attitude_transcriber.cpp)
ament_target_dependencies(ext_att_transcriber rclcpp roscopter_msgs rosflight_msgs)
target_link_libraries(ext_att_transcriber ${ament_LIBRARIES})
install(TARGETS ext_att_transcriber
  DESTINATION lib/${PROJECT_NAME})

# Controller
add_executable(controller
  src/controller/controller_ros.cpp
  src/controller/controller_state_machine.cpp
  src/controller/controller_cascading_pid.cpp
  src/controller/simple_pid.cpp)
ament_target_dependencies(controller rclcpp rclpy roscopter_msgs rosflight_msgs Eigen3)
target_link_libraries(controller ${ament_LIBRARIES} param_manager)
install(
  TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

# Trajectory Follower
add_executable(trajectory_follower
  src/navigation/trajectory_follower_ros.cpp
  src/navigation/trajectory_follower.cpp
  src/controller/simple_pid.cpp)
ament_target_dependencies(trajectory_follower rclcpp rclpy roscopter_msgs std_srvs rosflight_msgs Eigen3)
target_link_libraries(trajectory_follower ${ament_LIBRARIES} param_manager)
install(
  TARGETS trajectory_follower
  DESTINATION lib/${PROJECT_NAME})

# Path Manager
add_executable(path_manager
  src/navigation/path_manager_ros.cpp
  src/navigation/path_manager.cpp)
ament_target_dependencies(path_manager rclcpp rclpy roscopter_msgs std_srvs)
target_link_libraries(path_manager ${ament_LIBRARIES} param_manager)
install(
  TARGETS path_manager
  DESTINATION lib/${PROJECT_NAME})

# Path Planner
add_executable(path_planner
  src/navigation/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp roscopter_msgs std_srvs rosflight_msgs)
target_link_libraries(path_planner ${ament_LIBRARIES} param_manager ${YAML_CPP_LIBRARIES})
install(
  TARGETS path_planner
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(estimator_test test/estimator_test.cpp)
  ament_target_dependencies(estimator_test rclcpp geometry_msgs rosflight_msgs sensor_msgs Eigen3 roscopter_msgs)
  target_include_directories(estimator_test PUBLIC
    include
  )
  target_link_libraries(estimator_test estimation_lib param_manager) 

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

ament_package()
