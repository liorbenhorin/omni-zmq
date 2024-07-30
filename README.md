# Omniverse ZMQ Bridge

## Motivation

Omniverse is the most advanced simulator for vision-based systems and robots.
It has a great built-in support for ROS communication of sensory data, but not everyone is using ROS.
This extension is built to provide simple and performent method to communicate sensory and general data to/from the simulator, using only python.
The extention is built on top of [PyØMQ](https://pyzmq.readthedocs.io/en/latest/)

## Design

The program is made from a client and a server.
The client is the Omniverse application.
The server is a python app, running inside a docker container.
The server aim to mimic or act as a foundation for running vision models and build behviour logics that will be communicated back to the simulator, forming a SIL (Software in the loop).
Later this can be converted to run on edge device to form HIL (Hardware in the loop).

![alt text](exts/lbenhorin.nvidia.zmq_bridge/data/preview.png)

## Quick start

Clone this repo 
``` bash
git clone https://github.com/liorbenhorin/omni-zmq.git
cd omni-zmq
```

> This extension and sample files are desinged for Isaac SIM.
> It is leveraging the Isaac SIM API to simple the interaction with a 2DoF camera
> The user can run this extension on any other Kit-Based application, but they will
> need to remove the Isaac SIM API dependencies.


### Client (Omniverse Extension)

1. Launch Isaac SIM app.
2. Open the Window->Extensions from the top menu.
3. On the Extensions manager windows, open the settings from the "hamburger" button near the search field.
4. On the paths list, click the "+" button and add the path to where you cloned this repo + `/exts`.
5. After adding the path, you will see the `LBENHORIN NVIDIA ZMQ_BRIDGE` extension in the Third-Party tab, enable it + set it to Auto Load. 
6. Open the file `example_stage.usd` from the `assets` folder in this repo

### Server (Python app)

> The docker run command already introduces flags to enable NVIDIA CUDA support and GUI support.
> This is dependend on [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
> Also, the flags are written for Linux Ubuntu environment, and may need to be modified for Windows.

1. Build the docker image and run it 
```bash
cd omni-zmq-server
./build_server.sh
./run_server.sh
```
2. Inside the container, run the server 
```bash
python3 server.py
```

### Communication
On the Issac SIM side, click `reset world` button and then `Start RGB Streaming`

You should see the sensor data displayed on the server side.
You can use the arrow keys on the server side to rotate the camera, and the focal length slider.



## Next steps

After running this basic example it is time to learn how this extenstion is working and extand it.

First, it is recommended to use VScode as your IDE.
Start by linking the repo to your Kit app (In our case, it will be the path to where Isaac SIM is installed)

From the root of this repo, run
`./link_app.sh --path <your isaac sim install path>`

After a successful link, please restart VSCode.

The extenstions in Omniverse are leveraging hot-reload, meaning every time you will save a file under the `/exts` folder, Kit will reload the extension, be minded of it.

The docker run script have mounted the `/omni-zmq-server/src` folder, so you can make changes to the code while to container is running.


> Try to install and run a vision model like YOLO inside the container!