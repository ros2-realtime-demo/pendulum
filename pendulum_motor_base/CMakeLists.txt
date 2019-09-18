cmake_minimum_required(VERSION 3.5)

project(pendulum_motor_base)

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

include_directories(
  include
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_library(pendulum_motor_base SHARED
        src/pendulum_motor_base.cpp)

target_compile_definitions(pendulum_motor_base
        PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pendulum_motor_base
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_msgs"
        "rclcpp"
        "rclcpp_components")
rclcpp_components_register_nodes(pendulum_motor_base "pendulum::MotorBase")


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
          "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
        pendulum_motor_base
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
