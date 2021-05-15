FROM ros2realtimedemo/pendulum:base-rolling-tracers

MAINTAINER Lander Usategui <lander dot usategui at gmail dot com>

# After apt install sudo
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Set disable_coredump false' >> /etc/sudo.conf

COPY env.sh /etc/profile.d/ade_env.sh
COPY entrypoint /ade_entrypoint
COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & while wait ${!}; test $? -ge 128; do true; done"]
