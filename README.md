# Pendulum

[![pipeline status](https://gitlab.com/LanderU/pendulum/badges/master/pipeline.svg)](https://gitlab.com/LanderU/pendulum/commits/master)
[![coverage report](https://gitlab.com/LanderU/pendulum/badges/master/coverage.svg)](https://gitlab.com/LanderU/pendulum/commits/master)


### Project description

The aim of this project is show the real-time capabilities using ROS2 framework. This project is based on the [previous](https://index.ros.org/doc/ros2/Tutorials/Real-Time-Programming/) work made by the Open Robotics. The project uses these packages as a base.

#### Project architecture

```bash
├── [ 98K]  Doxyfile
├── [ 11K]  LICENSE
├── [4.0K]  pendulum_bringup
│   ├── [ 297]  CMakeLists.txt
│   ├── [4.0K]  config
│   │   └── [6.2K]  pendulum.rviz
│   ├── [4.0K]  launch
│   │   └── [1.3K]  pendulum_bringup.launch.py
│   └── [ 875]  package.xml
├── [4.0K]  pendulum_controller
│   ├── [4.0K]  pendulum_controller_node
│   │   ├── [2.5K]  CMakeLists.txt
│   │   ├── [4.0K]  include
│   │   │   └── [4.0K]  pendulum_controller_node
│   │   │       ├── [1.9K]  pendulum_controller.hpp
│   │   │       ├── [7.0K]  pendulum_controller_node.hpp
│   │   │       └── [1.9K]  visibility_control.hpp
│   │   ├── [1.3K]  package.xml
│   │   └── [4.0K]  src
│   │       └── [ 10K]  pendulum_controller_node.cpp
│   └── [4.0K]  pendulum_controllers
│       ├── [1.5K]  CMakeLists.txt
│       ├── [4.0K]  include
│       │   └── [4.0K]  pendulum_controllers
│       │       └── [2.9K]  full_state_feedback_controller.hpp
│       ├── [1018]  package.xml
│       └── [4.0K]  src
│           └── [2.2K]  full_state_feedback_controller.cpp
├── [4.0K]  pendulum_demo
│   ├── [3.8K]  CMakeLists.txt
│   ├── [1.7K]  package.xml
│   ├── [ 793]  README.md
│   ├── [4.0K]  scripts
│   │   ├── [ 210]  activate_pendulum.bash
│   │   ├── [ 114]  deactivate_pendulum.bash
│   │   └── [ 398]  move_pendulum.bash
│   └── [4.0K]  src
│       ├── [8.2K]  pendulum_controller_standalone.cpp
│       ├── [ 10K]  pendulum_demo.cpp
│       └── [8.6K]  pendulum_driver_standalone.cpp
├── [4.0K]  pendulum_description
│   ├── [ 203]  CMakeLists.txt
│   ├── [ 623]  package.xml
│   └── [4.0K]  urdf
│       └── [3.7K]  pendulum.urdf
├── [4.0K]  pendulum_driver
│   ├── [4.0K]  pendulum_driver
│   │   ├── [2.5K]  CMakeLists.txt
│   │   ├── [4.0K]  include
│   │   │   └── [4.0K]  pendulum_driver
│   │   │       ├── [2.3K]  pendulum_driver_interface.hpp
│   │   │       ├── [6.8K]  pendulum_driver_node.hpp
│   │   │       └── [1.8K]  visibility_control.hpp
│   │   ├── [1.3K]  package.xml
│   │   └── [4.0K]  src
│   │       └── [ 10K]  pendulum_driver_node.cpp
│   └── [4.0K]  pendulum_simulation
│       ├── [1.5K]  CMakeLists.txt
│       ├── [4.0K]  include
│       │   └── [4.0K]  pendulum_simulation
│       │       ├── [3.7K]  pendulum_simulation.hpp
│       │       └── [2.9K]  runge_kutta.hpp
│       ├── [1019]  package.xml
│       └── [4.0K]  src
│           └── [4.6K]  pendulum_simulation.cpp
├── [4.0K]  pendulum_msgs_v2
│   ├── [ 753]  CMakeLists.txt
│   ├── [4.0K]  msg
│   │   ├── [ 118]  ControllerStats.msg
│   │   ├── [ 104]  PendulumCommand.msg
│   │   ├── [ 104]  PendulumState.msg
│   │   ├── [  92]  PendulumStats.msg
│   │   ├── [ 231]  Rusage.msg
│   │   ├── [ 155]  TimerStats.msg
│   │   └── [  46]  TopicStats.msg
│   └── [ 920]  package.xml
├── [4.0K]  pendulum_teleop
│   ├── [1.5K]  CMakeLists.txt
│   ├── [4.0K]  include
│   │   └── [4.0K]  pendulum_teleop
│   │       ├── [2.7K]  lifecycle_service_client.hpp
│   │       └── [2.1K]  pendulum_manager.hpp
│   ├── [1.1K]  package.xml
│   └── [4.0K]  src
│       ├── [4.6K]  lifecycle_service_client.cpp
│       ├── [1.6K]  pendulum_manager.cpp
│       └── [3.4K]  pendulum_teleop.cpp
├── [4.0K]  pendulum_tools
│   ├── [1.3K]  CMakeLists.txt
│   ├── [4.0K]  include
│   │   └── [4.0K]  pendulum_tools
│   │       └── [1.4K]  timing_analyzer.hpp
│   ├── [ 741]  package.xml
│   └── [4.0K]  src
│       └── [1.4K]  timing_analyzer.cpp
└── [ 571]  README.md
```

#### Install instructions

In order to get the project ready, first you need to install [ROS2](https://index.ros.org/doc/ros2/Installation/Dashing/) and [rosdep](http://wiki.ros.org/rosdep).


```bash
source /opt/ros/dashing/setup.bash
mkdir -p ~/pendulum_ws/src
cd ~/pendulum_ws/src
git clone https://gitlab.com/LanderU/pendulum/ # Change me with valid github URL @LanderU
cd ~/pendulum_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro dashing -y
colcon build --merge-install # OR colcon build --symlink-install
```

#### Run instructions

```bash
fill me!
```

#### How to contribute

Make a fork using github, and make your changes. This project uses the same guidelines as all the officials ROS2 packages, make sure that your code is compliant with cppcheck, uncrustify...
After this, submit a PR and we will review as soon as possible.

#### Issues and feature request

Please, use the templates and make sure that you labeled them properly. Also, consider to open a PR using the template to fix the issue or to implement a feature.
