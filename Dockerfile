FROM ros:foxy

RUN mkdir -p /root/ros2_ws/src
RUN mkdir -p /root/ros2_symlink/src

COPY ./ /tmp/pendulum

RUN /tmp/pendulum/github-actions.sh
