# Inverted pendulum demo

## Disclaimer

This is work in progress and should be considered just as a proof of concept on how to leverage ROS
 2 features to use in real-time applications. Any feedback and help to improve the project is
  appreciated.

**Foxy branch is under active development. The current status can be checked here: 
https://github.com/ros2-realtime-demo/pendulum/issues/35**

## Build status

![Build Status](https://github.com/ros2-realtime-demo/pendulum/workflows/build/badge.svg?branch=foxy) [![License](https://img.shields.io/badge/license-Apache%202-blue)]() ![GitHub issues](https://img.shields.io/github/issues/ros2-realtime-demo/pendulum)

## Table of content

* [Inverted pendulum demo](#inverted-pendulum-demo)
    * [Project description](#project-description)
    * [Install instructions](#install-instructions)
    * [How to run the demo](#how-to-run-the-demo)
    * [How to configure real-time settings](#rtdemo)
    * [How to measure real-time performance metrics](#rtmetrics)
    * [How to contribute](#how-to-contribute)
    * [Issues and feature request](#issues-and-feature-request)

## Project description

The aim of this project is to show the real-time capabilities using the ROS2 framework. This
 project is based on the [previous](https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html)
  work made by the Open Robotics. The project uses these packages as a base.

This is the motivation of using an inverted pendulum for a real-time demo:

>A classic example of a controls problem commonly solved by real-time computing is balancing an inverted pendulum. If the controller blocked for an unexpectedly long amount of time, the pendulum would fall down or go unstable. But if the controller reliably updates at a rate faster than the motor controlling the pendulum can operate, the pendulum will successfully adapt react to sensor data to balance the pendulum.

A more detailed description of demo the project design be found here: [Design Article](docs/design.md)

## Install instructions

 Several ways to install the pendulum demo project are provided depending on the user needs. 
 
- [ADE based installation](docs/installation.md#ade-based-installation): This provides a 
pre-configured Docker image with everything ready to be used. This is the recommended option to
 quickly test the demo. It is also useful to have development environment to add new features.
- [Installation from source](docs/installation.md#installation-from-source): provides instructions
 to build all the required ROS 2 packages from source.
- [Crosscompilation](docs/installation.md#cross-compile): provides instructions to
 crosscompile the demo for an embedded device.
- [Pre-configured Raspberry PI image](docs/installation.md#raspberry-pi-image): provides a
 Raspberry PI with everything configured in a Linux Real-time (RT-PREEMPT patch).

## How to run the demo

### Source your workspace

If installed as an ADE image:

```bash
source /opt/ros/foxy/setup.bash
```

If installed from source:

```bash
cd ~/pendulum_ws
source ./install/setup.bash
```

### Launch the demo

A complete explanation of all the possible options to run the demo can be found in the tutorial
 document:
 - [Tutorial](docs/tutorial.md)

For a quick test, launch the demo with rviz enabled:

```bash
ros2 launch pendulum_bringup pendulum_bringup.launch.py rviz:=True
```

If everything went well you should see the inverted pendulum being controlled in rviz:

![pendulum_rviz](docs/images/pendulum_rviz.gif)


## How to configure real-time settings<a name="rtdemo"></a>

Instructions to configure the demo real-time settings can be found [here](docs/real_time_tutorial.md).

## How to measure real-time performance metrics<a name="rtmetrics"></a>

TODO

## How to contribute

Make a fork using github, and make your changes. This project uses the same guidelines as all the
 officials ROS 2 packages, make sure that your code is compliant with cppcheck, uncrustify...
After this, submit a PR and we will review as soon as possible.

## Issues and feature request

Please, use the [templates](https://github.com/ros2-realtime-demo/pendulum/issues/new/choose) and
 make sure that you labeled them properly. Also, consider to open a PR to fix the issue or to
  implement a feature.
