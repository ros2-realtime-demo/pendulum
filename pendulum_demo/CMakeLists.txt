cmake_minimum_required(VERSION 3.5)

project(pendulum_demo)

set(OPTIONAL_AMENT_DEPENDENCES)
set(OPTIONAL_LIBRARIES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rttest)
find_package(rcutils)
find_package(pendulum_motor_driver REQUIRED)
find_package(pendulum_controller REQUIRED)
find_package(pendulum_motor_node REQUIRED)
find_package(pendulum_controller_node REQUIRED)

find_package(osrf_testing_tools_cpp)
if(${osrf_testing_tools_cpp_FOUND})
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    list(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DPENDULUM_DEMO_MEMORYTOOLS_ENABLED)
endif()

include_directories(
        ${lifecycle_msgs_INCLUDE_DIRS}
        ${rclcpp_lifecycle_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${osrf_testing_tools_cpp_INCLUDE_DIR})

add_executable(pendulum_manual_composition
  "src/pendulum_manual_composition.cpp")

ament_export_include_directories(include)

ament_target_dependencies(pendulum_manual_composition
        "rclcpp"
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_msgs"
        "pendulum_motor_driver"
        "pendulum_controller"
        "pendulum_motor_node"
        "pendulum_controller_node"
        "rclcpp"
        "rttest"
        "rcutils"
        "rclcpp_components"
        ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(pendulum_manual_composition ${OPTIONAL_LIBRARIES})

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
    ament_environment_hooks(
            "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
        pendulum_manual_composition
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
