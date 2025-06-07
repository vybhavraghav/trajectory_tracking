cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(trajectory_tracking_velocity_node
  src/trajectory_tracking_velocity_controlled.cpp)
target_include_directories(trajectory_tracking_velocity_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_tracking_velocity_node
  rclcpp
  mavros_msgs
  tf2
  geometry_msgs
  nav_msgs
  )  

add_executable(trajectory_tracking_attitude_node
  src/trajectory_tracking_attitude_controlled.cpp)
target_link_libraries(trajectory_tracking_attitude_node Eigen3::Eigen)
target_include_directories(trajectory_tracking_attitude_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_tracking_attitude_node
  rclcpp
  mavros_msgs
  tf2
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
  )  

add_executable(waypoint_navigation_node
  src/waypoint_navigation.cpp)
target_include_directories(waypoint_navigation_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_navigation_node
  rclcpp
  mavros_msgs
  tf2
  geometry_msgs
  nav_msgs
  visualization_msgs
  )  


add_executable(waypoint_navigation_attitude_control_node
  src/waypoint_navigation_attitude_controlled.cpp)
target_link_libraries(waypoint_navigation_attitude_control_node Eigen3::Eigen)
target_include_directories(waypoint_navigation_attitude_control_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_navigation_attitude_control_node
  rclcpp
  mavros_msgs
  tf2
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
  )  


install(
  TARGETS trajectory_tracking_velocity_node trajectory_tracking_attitude_node waypoint_navigation_node waypoint_navigation_attitude_control_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
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
