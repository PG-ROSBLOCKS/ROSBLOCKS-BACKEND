FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y git wget python3-pip vim docker.io
RUN pip3 install setuptools==58.2.0

RUN git clone https://github.com/divyanshuraj6815/experiments.git
RUN git config --global user.email "your.mail@gmail.com"
RUN git config --global user.name "username"

ENV DISPLAY=novnc:0.0