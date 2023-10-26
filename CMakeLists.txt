cmake_minimum_required(VERSION 3.8)
project(arc_rviz_plugins)

# -------------------------------------
# Default to C99 & C++14
# -------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# -------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------
# find dependencies
# -------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)

# -------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/close_all_tool.cpp
  src/open_all_tool.cpp
)
# -------------------------------------
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rviz_common
  rviz_rendering
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
