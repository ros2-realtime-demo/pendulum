# Install instructions

## ADE based installation

TODO: describe how the ADE image is structured and which features already includes (ROS 2traces
 enabled, real-time limits configured, etc)  

Instructions taken from: https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation.html

[ADE](https://ade-cli.readthedocs.io/en/latest/index.html) is a modular Docker-based tool to ensure that all developers in a project have a common,
 consistent development environment.

Follow the install instructions, which are reproduced here for convenience:

1. Verify that the requirements listed
 [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements) are fulfilled
2. Download the statically-linked binary from the
 [Releases](https://gitlab.com/ApexAI/ade-cli/-/releases) page of the ade-cli project
3. Name the binary ade and install it in your PATH (on Ubuntu, `/usr/local/bin` is recommended)
4. Make the binary executable: `chmod +x ade`
5. Check that it is installed:

```bash
$ which ade
/path/to/ade
$ ade --version
<version>
```

### Setup ADE home and project checkout

ADE needs a directory on the host machine which is mounted as the user's home directory within
 the container. The directory is populated with dotfiles, and must be different than the
  user's home directory outside of the container. In the event ADE is used for multiple,
   projects it is recommended to use dedicated adehome directories for each project.

ADE looks for a directory containing a file named `.adehome` starting with the current
 working directory and continuing with the parent directories to identify the ADE home
  directory to be mounted.

```bash
$ mkdir adehome
$ cd adehome
$ touch .adehome
```

For ADE to function, it must be properly configured. The pendulum demo provides an `.aderc` file
 which is expected to exist in the current working directory, or in any parent directory.

```
$ cd adehome
$ mkdir -p pendulum_ws/src
$ cd pendulum_ws/src
$ git clone git@github.com:ros2-realtime-demo/pendulum.git
```

How to build
```
cd adehome
$ ade --rc pendulum_ws/src/pendulum/ade/.aderc start --update --enter
ade$ source /opt/ros/foxy/setup.bash
ade$ cd pendulum_ws
ade$ colcon build
```

## Installation from source

TODO

## Cross compile

TODO

## Raspberry PI image

TODO
