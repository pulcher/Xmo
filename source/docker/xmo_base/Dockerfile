FROM ubuntu:20.04
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DOMAIN_ID 27

COPY ./requirements.txt .
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive \
    TZ=America/Chicago \
    apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    git \
    vim \
    inetutils-ping \
    usbutils \
    i2c-tools \
    libi2c-dev \
    python3-rpi.gpio \
    v4l-utils \
    ffmpeg \
    python3-pip

RUN pip3 install -r requirements.txt

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
# RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-desktop
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-ros-base ros-galactic-teleop-twist-joy ros-galactic-image-tools
