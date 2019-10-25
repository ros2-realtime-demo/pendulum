FROM ros:dashing

RUN mkdir -p ~/ros2_ws/src
RUN mkdir -p ~/ros2_symlink/src

COPY . /tmp/pendulum

RUN /tmp/pendulum/travis.sh
