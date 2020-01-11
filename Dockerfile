FROM ros:eloquent

RUN mkdir -p /root/ros2_ws/src
RUN mkdir -p /root/ros2_symlink/src

COPY ./ /tmp/pendulum

RUN /tmp/pendulum/travis.sh
