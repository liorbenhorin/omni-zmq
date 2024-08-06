FROM ubuntu:22.04


RUN apt-get update
RUN apt-get install pip libgl1-mesa-glx libglib2.0-0 -y 


RUN pip install dearpygui zmq opencv-python numpy==1.23.5

RUN pip install torch torchvision torchaudio

WORKDIR /omni-zmq-server/src
ADD ./src /omni-zmq-server/src

