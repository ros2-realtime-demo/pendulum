#!/bin/bash
: '
Author: Lander Usategui, lander dot usategui at gmail dot com
'

set -e

# Export colors
export RED="\033[31m"
export GREEN="\033[32m"
export YELLOW="\033[33m"
export BLUE="\033[34m"
export PURPLE="\e[0;35m"
export RESET="\e[0m"

# Paths
export WS="/root/ros2_ws"
export WS_SYMLINK="/root/ros2_symlink"
export ROS2_DISTRO="foxy"

function prepare_ws()
{
  echo -e "${YELLOW}Preparing work spaces${RESET}"
  cd ${WS} || exit
  cp -r /tmp/pendulum src
  cd ${WS_SYMLINK} || exit
  cp -r /tmp/pendulum src
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}prepare_ws failed${RESET}"
    exit $result
  else
    echo -e "${BLUE}prepare_ws function successfully finished${RESET}"
  fi
}

function run_rosdep()
{
  echo -e "${YELLOW}Making sure that everything is installed${RESET}"
  cd ${WS} || exit
  apt update -qq && rosdep update
  rosdep install -q -y --from-paths . --ignore-src --rosdistro \
                                  "${ROS2_DISTRO}" \
                                   --as-root=apt:false || true
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}run_rosdep failled${RESET}"
    exit $result
  else
    echo -e "${BLUE}All dependencies successfully installed${RESET}"
  fi
}

function compile_ws()
{
  # shellcheck source=/dev/null
  source /opt/ros/"${ROS2_DISTRO}"/setup.bash
  cd ${WS} || exit
  echo -e "${YELLOW}###### Packages to be compiled ######${RESET}"
  echo -e "${PURPLE}"
  colcon list --names-only
  echo -e "${RESET}"
  echo -e "${YELLOW}Compile the WS for ROS2${RESET}"
  colcon build --merge-install
  echo -e "${YELLOW}Testing WS${RESET}"
  if ! colcon test --merge-install --return-code-on-test-failure; then
    colcon test-result --verbose
    exit 1
  else
    echo -e "${BLUE}WS compiled successfully${RESET}"
  fi
}

function compile_ws_symlink()
{
  # shellcheck source=/dev/null
  source /opt/ros/"${ROS2_DISTRO}"/setup.bash
  cd ${WS_SYMLINK} || exit
  echo -e "${YELLOW}###### Packages to be compiled ######${RESET}"
  echo -e "${PURPLE}"
  colcon list --names-only
  echo -e "${RESET}"
  echo -e "${YELLOW}Compile the WS for ROS2 using --symlink-install${RESET}"
  colcon build --symlink-install
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}Error compiling the WS_SYMLINK${RESET}"
    exit $result
  else
    echo -e "${BLUE}WS_SYMLINK compiled successfully${RESET}"
  fi
}

prepare_ws
run_rosdep
compile_ws
compile_ws_symlink
#If we're here everything is okay
echo -e "${GREEN}All steps successfully completed${RESET}"
