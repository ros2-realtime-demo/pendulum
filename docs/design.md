## Pendulum demo design

### Overview

This is a rework of the existing pendulum demo in ros2/demos. In this project we want to create a new version that updates some the new features of ROS 2 such as compositions, lifecycle, new QoS and real-time support.

The project is structured in a modular way so each node component can be tested and extended individually. We also decouple the actual controller implementation of the controller node so it is easy to implement new kind of controllers. This is the same approach that `ros_control` uses. In the same way we decouple the actual motor base implementation so it is possible to simulate the robot or to communicate with a real one.  

### Real-time use case

One of the main goals of this demo is to show a representative example of a real-time capable ROS 2 based application. In order to demonstrate these capabilities we can start with the following points:

* Show no memory page faults when nodes are active. Make use of lifecycle activate/deactivate to show the stats.
* Assert there is no memory allocation during runtime. Make use of the [OSRF memory tools](https://github.com/osrf/osrf_testing_tools_cpp). Take [`performance_test`](https://github.com/ros2/performance_test) as a reference.
* Use deadline QoS. Store and show the statistics. Show that there are no missing deadlines.
* Add a dynamic analyzer to measure additional statistics (latencies, context switches, etc). For example the [ROS 2 Bosch lttng](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing) package.
* Think about using experimental executors more suitable for real-time (waitset, polling).
* Add liveliness QoS.
* Explore the use of callback groups.

**A cool usage would be to make use of lifecycle and show all the resulting stats when we deactivate a node.**

### TODO

General:

* [X] Change pub/sub QoS with a more real-time friendly one.
* [ ] Create launchfile using composition for the whole demo.
* [ ] Add doxygen documentation support.
* [ ] Create custom msg.
* [ ] Use tools to check memory allocation. i.e: valgrind memcheck or OSRF memory tools.

Controller:

* [ ] Add unit test to test controller basic interface behavior.
* [ ] Add unit test or executable to test a PID implementation.
* [ ] Use parameters to change controller configuration in runtime. For example to tune a PID.
* [ ] Improve current PID implemetation (add clamping).
* [ ] Create an additional controller i.e: LQR

Robot/Motor:

* [X] Add timer to publish position.
* [X] Add a simple pendulum model implementation.
* [ ] Add unit test to test the pendulum (motor) basic interface behavior.
* [ ] Add unit test or executable to test the dynamics of the model.

Visualization:

* [ ] Create a simple demo to show how to log the robot position. Make use of rosbag if possible.
* [ ] Create an example with rqt to visualize some plots with the robot position.
* [ ] Create a model for Visualization with rviz.

Teleoperation:

* [ ] Make a simple demo changing the pendulum setpoint with ROS 2 cli publisher.
* [ ] Make a simple demo changing the pendulum setpoint with ROS 2 executable.
* [ ] Make a package to teleoperate the pendulum from the keyboard.
* [ ] Make a package to teleoperate the pendulum using a bluetooth controller.

### Prototype

As a first step we are going to create a functional prototype that implements the same functionality existing the previous pendulum demo but with the new design. This prototype will contain the following components:

* `PendulumControllerNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the pendulum controller.
* `PendulumController`: Abstract class used by `PendulumControllerNode` to call the specific controller implementation.
* `TestPendulumController`: Class used to test `PendulumControllerNode`.
* `PIDController`:Simple PID controller implementation derived from `Controller`.
* `PendulumMotorNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the pendulum motor base.
* `PendulumMotor`: Abstract class used by `PendulumMotorNode` to call the specific motor base implementation.
* `PendulumMotorSim`: Simple motor base physics simulation from `PendulumMotor`.
* `TestPendulumMotor`: Class used to test `PendulumMotorNode`.
* `pendulum_demo`: Program that creates and executor, adds a `PendulumControllerNode` and a `PendulumMotorNode` nodes with a `PIDController` and a `PendulumMotorSim` instances and spins all the nodes.
