cmake_minimum_required(VERSION 3.8)
project(fsw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# navigation node
add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(
   navigation_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
)
target_link_libraries(navigation_node Eigen3::Eigen)

target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# guidance node
add_executable(guidance_node src/guidance_node.cpp)
ament_target_dependencies(
   guidance_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
)
target_link_libraries(guidance_node Eigen3::Eigen)

target_include_directories(guidance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# control node
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(
   control_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
)
target_link_libraries(control_node Eigen3::Eigen)

target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# actuator node
add_executable(actuator_node src/actuator_node.cpp)
ament_target_dependencies(
   actuator_node
   rclcpp
   interfaces
   std_msgs
   geometry_msgs
)
target_link_libraries(actuator_node Eigen3::Eigen)

target_include_directories(actuator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install targets
install(TARGETS
  navigation_node
  guidance_node
  control_node
  actuator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
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
