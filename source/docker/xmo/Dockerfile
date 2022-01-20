#FROM pulcher/xmo_base:latest
FROM xmo_base:latest
# override this with your preferred ID
ENV ROS_DOMAIN_ID 27

WORKDIR /root

COPY ./entrypoint.sh /root

#USER 0
#ENTRYPOINT /usr/bin/bash -c /root/entrypoint.sh
#ENTRYPOINT [ "/usr/bin/bash", "-c", "/root/entrypoint.sh" ]
ENTRYPOINT [ "/root/entrypoint.sh" ]
#CMD [ "bash" ]
