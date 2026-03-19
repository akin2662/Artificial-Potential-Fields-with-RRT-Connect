cmake_minimum_required(VERSION 3.8)
project(apf_rrtc_plugin)

# Use C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)   # for Python files
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # for custom service

# ── Generate custom service ──────────────────────────────────────────────────
# This generates C++ headers from srv/ComputePath.srv
# The generated header will be at:
#   apf_rrtc_plugin/srv/compute_path.hpp
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputePath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# We need the generated interfaces before building the C++ plugin
ament_export_dependencies(rosidl_default_runtime)

# ── Build the C++ plugin shared library ──────────────────────────────────────
add_library(${PROJECT_NAME}_plugin SHARED
  src/apf_rrtc_plugin.cpp
)

# Include directories — our header and generated service headers
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against all required libraries
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
)

# Link against our generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin
  "${cpp_typesupport_target}")

# Tell pluginlib where to find the plugin — reads plugin.xml
pluginlib_export_plugin_description_file(nav2_core plugin.xml)
install(CODE
  "file(WRITE
    \"\${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/nav2_core__pluginlib__plugin/${PROJECT_NAME}\"
    \"share/${PROJECT_NAME}/plugin.xml\"
  )"
)

# ── Install C++ library ───────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# ── Install Python package ────────────────────────────────────────────────────
install(DIRECTORY apf_rrtc_plugin/
  DESTINATION lib/python3.10/site-packages/apf_rrtc_plugin
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY apf_rrtc_plugin/
  DESTINATION local/lib/python3.10/dist-packages/apf_rrtc_plugin
  FILES_MATCHING PATTERN "*.py"
)

# Install Python scripts (executables)
install(PROGRAMS
  apf_rrtc_plugin/planner_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install other files ───────────────────────────────────────────────────────
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
