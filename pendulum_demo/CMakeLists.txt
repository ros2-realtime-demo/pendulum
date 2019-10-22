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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(pendulum_tools REQUIRED)
find_package(pendulum_msgs_v2 REQUIRED)
find_package(pendulum_simulation REQUIRED)
find_package(pendulum_controllers REQUIRED)
find_package(pendulum_driver REQUIRED)
find_package(pendulum_controller_node REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(osrf_testing_tools_cpp)
if(${osrf_testing_tools_cpp_FOUND})
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    list(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DPENDULUM_DEMO_MEMORYTOOLS_ENABLED)
endif()

find_package(tlsf_cpp)
if(${tlsf_cpp_FOUND})
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "tlsf_cpp")
    add_definitions(-DPENDULUM_DEMO_TLSF_ENABLED)
endif()

include_directories(
        ${lifecycle_msgs_INCLUDE_DIRS}
        ${pendulum_msgs_v2_INCLUDE_DIRS}
        ${rclcpp_lifecycle_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${osrf_testing_tools_cpp_INCLUDE_DIR}
        ${tlsf_cpp_INCLUDE_DIR}
        ${pendulum_tools_INCLUDE_DIR})

add_executable(pendulum_demo
  "src/pendulum_demo.cpp")

ament_export_include_directories(include)

ament_target_dependencies(pendulum_demo
        "rclcpp"
        "rclcpp_lifecycle"
        "rclcpp_components"
        "rcutils"
        "lifecycle_msgs"
        "pendulum_msgs_v2"
        "pendulum_simulation"
        "pendulum_controllers"
        "pendulum_driver"
        "pendulum_controller_node"
        "sensor_msgs"
        "pendulum_tools"
        ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(pendulum_demo ${OPTIONAL_LIBRARIES})

add_executable(pendulum_controller_standalone
  "src/pendulum_controller_standalone.cpp")

ament_export_include_directories(include)

ament_target_dependencies(pendulum_controller_standalone
        "rclcpp"
        "rclcpp_lifecycle"
        "rclcpp_components"
        "rcutils"
        "lifecycle_msgs"
        "pendulum_msgs_v2"
        "pendulum_controllers"
        "pendulum_controller_node"
        "sensor_msgs"
        "pendulum_tools"
        ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(pendulum_controller_standalone ${OPTIONAL_LIBRARIES})

add_executable(pendulum_driver_standalone
  "src/pendulum_driver_standalone.cpp")

ament_export_include_directories(include)

ament_target_dependencies(pendulum_driver_standalone
        "rclcpp"
        "rclcpp_lifecycle"
        "rclcpp_components"
        "rcutils"
        "lifecycle_msgs"
        "pendulum_msgs_v2"
        "pendulum_simulation"
        "pendulum_driver"
        "sensor_msgs"
        "pendulum_tools"
        ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(pendulum_driver_standalone ${OPTIONAL_LIBRARIES})


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
    ament_environment_hooks(
            "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
        pendulum_demo
        pendulum_controller_standalone
        pendulum_driver_standalone
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
