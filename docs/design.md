## Pendulum demo design

### Purpose

This is a rework of the existing pendulum demo in ros2/demos. In this project we want to create a new version that updates some the new features of ROS 2 such as compositions, lifecycle, new QoS and real-time support.

The project is structured in a modular way so each node component can be tested and extended individually. We also decouple the actual controller implementation of the controller node so it is easy to implement new kind of controllers. This is the same approach that `ros_control` uses. In the same way we decouple the actual driver implementation so it is possible to simulate the robot or to communicate with a real one.  

![rosgraph](images/rosgraph.png)

### Real-time

One of the main goals of this demo is to show a representative example of a real-time capable ROS 2 based application. In order to demonstrate these capabilities we can start with the following points:

* Show no memory page faults when nodes are active. Make use of lifecycle activate/deactivate to show the stats.
* Use deadline QoS. Store and show the statistics. Show that there are no missing deadlines.
* Add a dynamic analyzer to measure additional statistics (latencies, context switches, etc). For example the [ROS 2 Bosch lttng](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing) package.
* Think about using experimental executors more suitable for real-time (waitset, polling).
* Add liveliness QoS and other real-time friendly DDS QoS.
* Explore the use of callback groups.

### Pendulum control

Another goal of the demo is to provide a simple use case to develop and test with different controllers. For this reason, it would be interesing to add more controllers to the existing projects. For example a PID controller or a [LQG controller](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic%E2%80%93Gaussian_control) by adding a [kalman filter](https://en.wikipedia.org/wiki/Kalman_filter). It would interesting also to add new capabilities such as a [swing-up](https://youtu.be/hQK_3C6S4Ak?t=285) capable controller.

### Component overview

The first version of the demo contains the following components:

* `PendulumControllerNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the pendulum controller.
* `PendulumController`: Abstract class used by `PendulumControllerNode` to call the specific controller implementation.
* `FullStateFeedbackController`: A controller implementation derived from `Controller` based on a [full state feedback controller](https://en.wikipedia.org/wiki/Full_state_feedback).
* `PendulumDriverNode`: Class derived from `LifecycleNode` that implements the ROS 2 interface for the real or simulated pendulum.
* `PendulumDriverInterface`: Abstract class used by `PendulumDriverNode` to call the specific driver base implementation.
* `PendulumSimulation`: A class derived from PendulumDriverInterface that implements a simulation of a cart based pendulum.
* `pendulum_demo`: Program that creates and executor, adds a `PendulumControllerNode` and a `PendulumDriverNode` nodes with a `FullStateFeedbackController` and a `PendulumSimulation` instances and spins all the nodes. It provides also options to test different publishing periods, parameters and real-time settings.
* `pendulum_manager` An executable to manage the lifecycle node transitions. In the future this would be a rqt plugin with a control panel showing the status of the nodes and with buttons to transit the nodes between states.
* `pendulum_teleop` An executable to send setpoints to the controller using a joystick or a keyboard. (Not implemented yet).
