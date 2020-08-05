FROM ros:foxy

RUN apt-get update && \
    apt-get install -y \
        sudo \
        locales \
        tzdata \
  && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# After apt install sudo
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

COPY env.sh /etc/profile.d/ade_env.sh
COPY entrypoint /ade_entrypoint
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & while wait ${!}; test $? -ge 128; do true; done"]
