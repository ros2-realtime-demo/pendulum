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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pendulum2_msgs REQUIRED)
find_package(pendulum_tools REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    pendulum2_msgs
    pendulum_tools)

set(PENDULUM_DRIVER_LIB pendulum_driver)
add_library(${PENDULUM_DRIVER_LIB} SHARED
    src/pendulum_driver_node.cpp
    src/pendulum_driver.cpp
    src/pendulum_driver_config.cpp)

target_include_directories(${PENDULUM_DRIVER_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PENDULUM_DRIVER_LIB} ${dependencies})

rclcpp_components_register_nodes(${PENDULUM_DRIVER_LIB} "pendulum::Driver")

# Generate standalone node executable
set(PENDULUM_DRIVER_EXE "${PENDULUM_DRIVER_LIB}_exe")
add_executable(${PENDULUM_DRIVER_EXE} src/pendulum_driver_node_main.cpp)
target_link_libraries(${PENDULUM_DRIVER_EXE} ${PENDULUM_DRIVER_LIB})

ament_export_targets(export_${PENDULUM_DRIVER_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Node
  set(TEST_NODE_NAME "test_config")

  # "Unit" test
  set(CONFIG_GTEST test_config)

  ament_add_gtest(${CONFIG_GTEST}
    "test/test_config.cpp"
  )

  target_include_directories(${CONFIG_GTEST} PRIVATE include)
  target_link_libraries(${CONFIG_GTEST} pendulum_driver)
  ament_target_dependencies(${TEST_NODE_NAME}
    "rclcpp"
    "rclcpp_lifecycle"
    "pendulum2_msgs"
    "sensor_msgs"
  )

endif()

install(
    DIRECTORY include/
    DESTINATION include
)

install(TARGETS ${PENDULUM_DRIVER_LIB} ${PENDULUM_DRIVER_EXE}
    EXPORT export_${PENDULUM_DRIVER_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)

ament_package()
