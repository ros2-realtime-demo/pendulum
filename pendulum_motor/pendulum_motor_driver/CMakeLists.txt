cmake_minimum_required(VERSION 3.5)

project(pendulum_motor_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rttest)
find_package(rttest)
find_package(pendulum_motor_node)

include_directories(include)

add_executable(test_pendulum_motor
        test/test_pendulum_motor.cpp)

ament_export_include_directories(include)

ament_target_dependencies(test_pendulum_motor
        "rclcpp"
        "pendulum_msgs"
        "rttest"
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_motor_node"
        "rclcpp_components")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if (NOT WIN32)
    ament_environment_hooks(
            "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif ()


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
        test_pendulum_motor
        DESTINATION lib/${PROJECT_NAME})

ament_package()
