cmake_minimum_required(VERSION 3.5)

project(pendulum_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(PENDULUM_TOOLS_LIB pendulum_tools)

add_library(${PENDULUM_TOOLS_LIB} SHARED
  src/memory_lock.cpp
  src/rt_thread.cpp)

target_include_directories(${PENDULUM_TOOLS_LIB}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PENDULUM_TOOLS_LIB} "rclcpp")

ament_export_targets(export_${PENDULUM_TOOLS_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies("rclcpp")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PENDULUM_TOOLS_LIB}
    EXPORT export_${PENDULUM_TOOLS_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)

ament_package()
