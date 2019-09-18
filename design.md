## Pendulum demo design

### Overview

This is a rework of the existing pendulum demo in ros2/demos. In this project we want to create a new version that updates some the new features of ROS 2 such as compositions, lifecycle, new QoS and real-time support.

The project is structured in a modular way so each node component can be tested and extended individually. We also decouple the actual controller implementation of the controller node so it is easy to implement new kind of controllers. This is the same approach that ros_control uses. In the same way we decouple the actual motor base implementation so it is possible to simulate the robot or to communicate with a real one.  

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
