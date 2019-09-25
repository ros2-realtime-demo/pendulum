## Pendulum demo design

### Overview

This is a rework of the existing pendulum demo in ros2/demos. In this project we want to create a new version that updates some the new features of ROS 2 such as compositions, lifecycle, new QoS and real-time support.

The project is structured in a modular way so each node component can be tested and extended individually. We also decouple the actual controller implementation of the controller node so it is easy to implement new kind of controllers. This is the same approach that `ros_control` uses. In the same way we decouple the actual motor base implementation so it is possible to simulate the robot or to communicate with a real one.  

### TODO

General:

* [ ] Change pub/sub QoS with a more real-time friendly one.
* [ ] Create launchfile using composition for the whole demo.
* [ ] Add doxygen documentation support.
* [ ] Create custom msg.
* [ ] Use tools to check memory allocation. i.e: valgrind memcheck or OSRF memory tools.

Controller:

* [ ] Add unit test to test controller basic interface behavior.
* [ ] Add unit test or executable to test a PID implementation.
* [ ] Use parameters to change controller configuration in runtime. For example to tune a PID.
* [ ] Improve current PID implemetation (add clamping).
* [ ] Create an aditional controller i.e: LQR

Robot/Motor:

* [ ] Add timer to publish position.
* [ ] Add a simple pendulum model implementation.
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

* `ControllerNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the pendulum controller.
* `Controller`: Abstract class used by `ControllerNode` to call the specific controller implementation.
* `TestController`: Class used to test `ControllerNode`.
* `PIDController`:Simple PID controller implementation derived from `Controller`.
* `MotorBaseNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the pendulum motor base.
* `MotorBase`: Abstract class used by `MotorBaseNode` to call the specific motor base implementation.
* `MotorBaseSimpleSim`: Simple motor base physics simulation from `MotorBase`.
* `TestMotorBase`: Class used to test `MotorBaseNode`.
* `demo_main`: Program that creates and executor, adds a `ControllerNode` and a `MotorBaseNode` nodes with a `PIDController` and a `MotorBaseSimpleSim` instances and spins all the nodes.
