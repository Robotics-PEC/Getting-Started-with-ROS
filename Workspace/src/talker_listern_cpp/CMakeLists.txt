cmake_minimum_required(VERSION 3.8)
project(talker_listern_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(keyboard_talker_char keyboard_talker_char.cpp)
ament_target_dependencies(keyboard_talker_char rclcpp std_msgs)

install(TARGETS
  talker
  listener
  keyboard_talker_char
  DESTINATION lib/${PROJECT_NAME})

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
