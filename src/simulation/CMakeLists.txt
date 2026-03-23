cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interfaces REQUIRED)         # custom interface package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# simulator node
add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(
   simulator_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
   tf2
   tf2_ros
)
target_link_libraries(simulator_node Eigen3::Eigen)

target_include_directories(simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# display node
add_executable(display_node src/display_node.cpp)
ament_target_dependencies(
   display_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
   tf2
   tf2_ros
   visualization_msgs
   rviz_2d_overlay_msgs
)

target_include_directories(display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install targets
install(TARGETS
   simulator_node
   display_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
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
