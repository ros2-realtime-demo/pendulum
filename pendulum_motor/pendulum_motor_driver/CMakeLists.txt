cmake_minimum_required(VERSION 3.5)

project(pendulum_motor_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pendulum_ex_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rttest)

include_directories(include)

add_library(pendulum_motor_driver INTERFACE)

target_include_directories(pendulum_motor_driver
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#ament_export_interfaces(export_pendulum_motor_driver )
ament_export_include_directories(include)
#ament_export_libraries(pendulum_motor_driver HAS_LIBRARY_TARGET)

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
        pendulum_motor_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
