cmake_minimum_required(VERSION 3.8)
project(my_nav)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
set(dependencies
  rclcpp

  # rclcpp_lifecycle
  # rclcpp_components
  nav_msgs
  nav2_msgs
  yaml_cpp_vendor
  std_msgs
  nav2_util)

include_directories(
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS}
)

add_executable(map_publisher_node
  src/map_pub.cpp)
add_executable(global_planner_node
  src/global_planner.cpp)
add_executable(robot_control_node
  src/robot_controller.cpp)



# target_include_directories(map_publisher_node SYSTEM PRIVATE
#   /usr/include
#   /opt/ros/humble/include
# )
# target_include_directories(global_planner_node SYSTEM PRIVATE
#   /usr/include
#   /opt/ros/humble/include
# )

# target_include_directories(robot_control_node SYSTEM PRIVATE
#   /usr/include
#   /opt/ros/humble/include
# )

target_link_libraries(map_publisher_node
  # /usr/lib/libGraphicsMagick++.so
  # /usr/lib/libGraphicsMagick.so
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)
target_link_libraries(global_planner_node
  /usr/lib/libGraphicsMagick++.so
  # /usr/lib/libGraphicsMagick.so
)
target_link_libraries(robot_control_node
  /usr/lib/libGraphicsMagick++.so
  # /usr/lib/libGraphicsMagick.so
)

ament_target_dependencies(map_publisher_node
  ${dependencies}
)
ament_target_dependencies(global_planner_node
  ${dependencies}
  std_srvs
)
ament_target_dependencies(robot_control_node
  ${dependencies}
  tf2
  tf2_ros
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

install(TARGETS robot_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS map_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch maps

  DESTINATION share/${PROJECT_NAME}
)
ament_package()
