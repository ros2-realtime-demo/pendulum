cmake_minimum_required(VERSION 3.5)

project(pendulum_controller)

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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pendulum_msgs_v2 REQUIRED)
find_package(pendulum_tools REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    pendulum_msgs_v2
    pendulum_tools)

set(PENDULUM_CONTROLLER_LIB pendulum_controller)
add_library(${PENDULUM_CONTROLLER_LIB} SHARED
        src/pendulum_controller_node.cpp
        src/pendulum_controller.cpp)

target_include_directories(${PENDULUM_CONTROLLER_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PENDULUM_CONTROLLER_LIB} ${dependencies})

rclcpp_components_register_nodes(${PENDULUM_CONTROLLER_LIB} "pendulum::Controller")

# Generate standalone node executable
set(PENDULUM_CONTROLLER_EXE "${PENDULUM_CONTROLLER_LIB}_exe")
add_executable(${PENDULUM_CONTROLLER_EXE} src/pendulum_controller_node_main.cpp)
target_link_libraries(${PENDULUM_CONTROLLER_EXE} ${PENDULUM_CONTROLLER_LIB})

ament_export_targets(export_${PENDULUM_CONTROLLER_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Node
  set(TEST_NODE_NAME "test_constructors")

  # "Unit" test
  set(CONSTRUCTORS_GTEST test_constructors)

  ament_add_gtest(${CONSTRUCTORS_GTEST}
    "test/test_constructors.cpp"
  )

  target_include_directories(${CONSTRUCTORS_GTEST} PRIVATE include)
  target_link_libraries(${CONSTRUCTORS_GTEST} pendulum_controller)
  ament_target_dependencies(${TEST_NODE_NAME}
    "rclcpp"
    "rclcpp_lifecycle"
    "pendulum_msgs_v2"
    "sensor_msgs"
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PENDULUM_CONTROLLER_LIB} ${PENDULUM_CONTROLLER_EXE}
        EXPORT export_${PENDULUM_CONTROLLER_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
