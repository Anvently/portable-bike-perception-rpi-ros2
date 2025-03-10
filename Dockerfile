FROM ros:jazzy-ros-base

RUN apt-get install python3-pip
COPY ./requirements.txt /etc/python-requirements.txt
RUN /bin/sh pip install -r /etc/python-requirements

