cmake_minimum_required(VERSION 3.5)

project(pendulum_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pendulum_msgs_v2 REQUIRED)
find_package(pendulum_controller_node REQUIRED)

include_directories(
  include
  ${pendulum_msgs_v2_INCLUDE_DIRS})

add_library(pendulum_controllers SHARED
  src/full_state_feedback_controller.cpp)

target_include_directories(pendulum_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)
ament_export_libraries(pendulum_controllers)

ament_target_dependencies(pendulum_controllers
        "pendulum_controller_node"
        "pendulum_msgs_v2")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
          "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
        pendulum_controllers
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
