cmake_minimum_required(VERSION 3.8)
project(rm_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rm_message REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DrawFigure.srv"
  "srv/DrawShape.srv"
  "srv/DeleteLayer.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(rm_ui_component SHARED
  src/ui.cpp
)

add_library(rm_ui_debugger_component SHARED
  src/ui_debugger.cpp
)

target_include_directories(rm_ui_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_include_directories(rm_ui_debugger_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(rm_ui_component
  rclcpp
  rclcpp_components
  rm_message
)

ament_target_dependencies(rm_ui_debugger_component
  rclcpp
  rclcpp_components
  rm_message
  sensor_msgs
  cv_bridge
)

target_link_libraries(rm_ui_component
  "${cpp_typesupport_target}"
)

target_link_libraries(rm_ui_debugger_component
  ${OpenCV_LIBS}
)

add_executable(rm_ui_node
  src/main.cpp
)

add_executable(rm_ui_debugger_node
  src/ui_debugger_main.cpp
)

set_target_properties(rm_ui_node
  PROPERTIES OUTPUT_NAME rm_ui
)

set_target_properties(rm_ui_debugger_node
  PROPERTIES OUTPUT_NAME rm_ui_debugger
)

target_link_libraries(rm_ui_node
  rm_ui_component
)

target_link_libraries(rm_ui_debugger_node
  rm_ui_debugger_component
)

rclcpp_components_register_nodes(rm_ui_component
  "rm_ui::RmUi"
)
rclcpp_components_register_nodes(rm_ui_debugger_component
  "rm_ui::RmUiDebugger"
)
pluginlib_export_plugin_description_file(rclcpp_components rm_ui.xml)
pluginlib_export_plugin_description_file(rclcpp_components rm_ui_debugger.xml)

install(TARGETS rm_ui_node rm_ui_debugger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS rm_ui_component rm_ui_debugger_component
  EXPORT rm_ui_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES rm_ui.xml rm_ui_debugger.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(rm_ui_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rm_message
  sensor_msgs
  cv_bridge
  OpenCV
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS
    "-c" "${CMAKE_CURRENT_SOURCE_DIR}/../rm_manager/uncrustify_4space.cfg")
  set(ament_cmake_uncrustify_ADDITIONAL_EXCLUDE
    "example/ui.cpp"
    "example/ui.hpp")

  ament_add_gtest(test_rm_ui
    test/test_rm_ui.cpp
  )

  ament_add_gtest(test_rm_ui_debugger
    test/test_rm_ui_debugger.cpp
  )

  if(TARGET test_rm_ui)
    target_link_libraries(test_rm_ui
      rm_ui_component
      "${cpp_typesupport_target}"
    )
    ament_target_dependencies(test_rm_ui
      rclcpp
      rm_message
      sensor_msgs
    )
  endif()

  if(TARGET test_rm_ui_debugger)
    target_link_libraries(test_rm_ui_debugger
      rm_ui_debugger_component
    )
    ament_target_dependencies(test_rm_ui_debugger
      rclcpp
      rm_message
      sensor_msgs
    )
  endif()

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
