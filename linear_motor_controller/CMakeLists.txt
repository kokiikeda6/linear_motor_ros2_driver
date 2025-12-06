cmake_minimum_required(VERSION 3.8)
project(linear_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linear_motor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

add_executable(linear_motor_controller_node
  src/linear_motor_controller_node.cpp
)

target_include_directories(linear_motor_controller_node PUBLIC
  ${GPIOD_INCLUDE_DIRS}
)

target_link_libraries(linear_motor_controller_node
  ${GPIOD_LIBRARIES}
)

ament_target_dependencies(linear_motor_controller_node
  rclcpp
  linear_motor_msgs
)

add_executable(linear_motor_client_node
  src/linear_motor_client_node.cpp
)

ament_target_dependencies(linear_motor_client_node
  rclcpp
  linear_motor_msgs
)

install(TARGETS
  linear_motor_controller_node
  linear_motor_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
