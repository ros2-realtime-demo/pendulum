cmake_minimum_required(VERSION 3.5)

project(pendulum_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(pendulum_msgs_v2 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
set(PENDULUM_DRIVER_LIB pendulum_driver)
add_library(${PENDULUM_DRIVER_LIB} SHARED
src/pendulum_driver_node.cpp
src/pendulum_driver.cpp
src/pendulum_driver_config.cpp)

target_include_directories(${PENDULUM_DRIVER_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)
ament_export_libraries(${PENDULUM_DRIVER_LIB})

ament_target_dependencies(${PENDULUM_DRIVER_LIB}
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_msgs_v2"
        "rclcpp"
        "rclcpp_components"
        "sensor_msgs")
rclcpp_components_register_nodes(${PENDULUM_DRIVER_LIB}
    "pendulum::pendulum_driver::PendulumDriver")
target_link_libraries(${PENDULUM_DRIVER_LIB})

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
    ${PENDULUM_DRIVER_LIB}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
