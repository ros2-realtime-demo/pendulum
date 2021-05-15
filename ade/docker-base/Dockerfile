ARG OS_VERSION
FROM $OS_VERSION

MAINTAINER Lander Usategui <lander dot usategui at gmail dot com>

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

ENV ROS_VERSION rolling

WORKDIR /root/ros2_$ROS_VERSION

RUN apt-get update \
  && echo 'Etc/UTC' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get install -y \
    firefox \
    sudo \
    locales \
    tzdata \
    libogre-1.12 \
    htop \
    stress-ng \
    rt-tests \
    vim \
    nano \
    heaptrack \
    bash-completion \
    openssh-client \
    lcov \
    strace \
    man \
    gdb \
    doxygen \
    curl \
    strace \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
  && locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list \
  && apt-add-repository ppa:lttng/ppa \
  && apt-get update \
  && apt-get install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    lttng-tools \
    lttng-modules-dkms \
    babeltrace \
    liblttng-ust-dev \
    python3-babeltrace \
    python3-lttng \
  && python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pandas \ 
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    bokeh \
    jupyter \
  && apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
  && mkdir -p /root/ros2_$ROS_VERSION/src \
  && wget https://raw.githubusercontent.com/ros2/ros2/$ROS_VERSION/ros2.repos \
  && vcs import src < ros2.repos \
  && git clone -b $ROS_VERSION https://gitlab.com/ApexAI/apex_test_tools src/apex_test_tools \
  && rosdep init \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src --rosdistro $ROS_VERSION -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" \
  && mkdir -p /opt/ros/$ROS_VERSION \
  && colcon build --merge-install --install-base /opt/ros/$ROS_VERSION/ \
  && rm -rf /var/lib/apt/lists/* \
  && rm -rf /root/ros2_$ROS_VERSION
