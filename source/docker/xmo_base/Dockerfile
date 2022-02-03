FROM ubuntu:20.04
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DOMAIN_ID 27

RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    git \
    vim \
    inetutils-ping \
    usbutils \
    i2c-tools \
    libi2c-dev \
    python3-pip

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-desktop

RUN pip3 install adafruit-pca9685
RUN pip3 install adafruit-circuitpython-servokit
RUN pip3 install smbus
RUN pip3 install smbus2
RUN pip3 install requests
