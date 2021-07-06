cmake_minimum_required(VERSION 3.5)

project(pendulum_tests)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pendulum_controller REQUIRED)
find_package(pendulum_driver REQUIRED)

if(BUILD_TESTING)
  find_package(apex_test_tools REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  apex_test_tools_add_gtest(test_pendulum_memory_allocations test/test_pendulum_memory_allocations.cpp)
  target_link_libraries(test_pendulum_memory_allocations
      pendulum_controller::pendulum_controller
      pendulum_driver::pendulum_driver)

  find_package(launch_testing_ament_cmake)
  add_launch_test(
      test/pendulum_demo_autostart.test.py
      TARGET pendulum_demo_autostart.test
      ENV
      TIMEOUT 30
  )
  add_launch_test(
      test/pendulum_demo_lifecycle.test.py
      TARGET pendulum_demo_lifecycle.test
      ENV
      TIMEOUT 30
  )
endif()

install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PENDULUM_DEMO_EXE}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)

ament_package()
