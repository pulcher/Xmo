#FROM pulcher/xmo_base:latest
FROM xmo_base:latest
# override this with your preferred ID
ENV ROS_DOMAIN_ID 27

WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b galactic-devel
WORKDIR /root/dev_ws

RUN apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro galactic -y
RUN apt install python3-colcon-common-extensions -y

WORKDIR /root

COPY ./entrypoint.sh /root

#USER 0
#ENTRYPOINT /usr/bin/bash -c /root/entrypoint.sh
#ENTRYPOINT [ "/usr/bin/bash", "-c", "/root/entrypoint.sh" ]
ENTRYPOINT [ "/root/entrypoint.sh" ]
#CMD [ "bash" ]
