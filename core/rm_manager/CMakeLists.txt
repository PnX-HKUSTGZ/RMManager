cmake_minimum_required(VERSION 3.22)
project(rm_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rm_message REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(rm_manager_core
  src/frame_parser.cpp
  src/msg_publisher.cpp
  src/rm_manager.cpp
  src/util.cpp)

ament_target_dependencies(rm_manager_core rclcpp rm_message Boost std_msgs)

target_include_directories(rm_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(rm_manager src/main.cpp)
target_link_libraries(rm_manager rm_manager_core)
ament_target_dependencies(rm_manager rclcpp)

install(TARGETS rm_manager rm_manager_core
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS
    "-c" "${CMAKE_CURRENT_SOURCE_DIR}/uncrustify_4space.cfg")
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_msg_publisher
    test/test_msg_publisher.cpp)
  if(TARGET test_msg_publisher)
    target_link_libraries(test_msg_publisher rm_manager_core)
    ament_target_dependencies(test_msg_publisher rclcpp rm_message std_msgs)
    target_include_directories(test_msg_publisher PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_package()
