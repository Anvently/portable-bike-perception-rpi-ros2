FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y python3-pip screen
COPY ./requirements.txt /etc/python-requirements.txt
RUN pip install --break-system-packages -r /etc/python-requirements.txt

