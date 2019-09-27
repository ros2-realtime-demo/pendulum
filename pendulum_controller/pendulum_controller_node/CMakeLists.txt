cmake_minimum_required(VERSION 3.5)

project(pendulum_controller_node)

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
find_package(rttest REQUIRED)
find_package(pendulum_controller REQUIRED)

include_directories(
  include
  ${pendulum_controller_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_library(pendulum_controller_node SHARED
        src/pendulum_controller_node.cpp)

target_include_directories(pendulum_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(pendulum_controller_node
        PRIVATE "COMPOSITION_BUILDING_DLL")

ament_export_include_directories(include)
ament_export_libraries(pendulum_controller_node)

ament_target_dependencies(pendulum_controller_node
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_msgs"
        "rclcpp"
        "pendulum_controller"
        "rclcpp_components")
rclcpp_components_register_nodes(pendulum_controller_node "pendulum::Controller")

add_executable(test_pendulum_controller
        test/test_pendulum_controller.cpp
        src/pendulum_controller_node.cpp)

ament_target_dependencies(test_pendulum_controller
        "rclcpp"
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        "pendulum_msgs"
        "pendulum_controller"
        "rttest"
        "rclcpp"
        "rclcpp_components")

target_link_libraries(test_pendulum_controller pendulum_controller_node)


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
        pendulum_controller_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        test_pendulum_controller
        DESTINATION lib/${PROJECT_NAME})


ament_package()
