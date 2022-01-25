FROM ubuntu:20.04
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DOMAIN_ID 27

RUN apt update && apt install -y curl gnupg2 lsb-release git vim inetutils-ping usbutils
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-desktop

WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b galactic-devel
WORKDIR /root/dev_ws

RUN apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro galactic -y
RUN apt install python3-colcon-common-extensions -y

#COPY ros2_entrypoint.sh /root/.
#ENTRYPOINT ["/root/ros2_entrypoint.sh"]
#CMD ["bash"]

