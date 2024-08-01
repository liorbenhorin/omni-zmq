docker run --gpus all --network host \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -e XAUTHORITY=$XAUTHORITY \
       -v $XAUTHORITY:$XAUTHORITY \
       -v ./src:/omni-zmq-server/src \
       -it --rm \
        omni-zmq-server bash
