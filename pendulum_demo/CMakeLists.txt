cmake_minimum_required(VERSION 3.5)

project(pendulum_demo)

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

set(PENDULUM_DEMO_EXE pendulum_demo)

add_executable(${PENDULUM_DEMO_EXE} "src/pendulum_demo.cpp")
target_link_libraries(${PENDULUM_DEMO_EXE}
    pendulum_controller::pendulum_controller
    pendulum_driver::pendulum_driver)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

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
