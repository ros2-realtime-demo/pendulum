cmake_minimum_required(VERSION 3.5)

project(pendulum_utils)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(PENDULUM_UTILS_LIB pendulum_utils)

add_library(${PENDULUM_UTILS_LIB} SHARED
  src/memory_lock.cpp
  src/rt_thread.cpp)

target_include_directories(${PENDULUM_UTILS_LIB}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PENDULUM_UTILS_LIB} "rclcpp")

ament_export_targets(export_${PENDULUM_UTILS_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    "rclcpp"
    "rclcpp_lifecycle"
    "lifecycle_msgs"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PENDULUM_UTILS_LIB}
    EXPORT export_${PENDULUM_UTILS_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)

ament_package()
