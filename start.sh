#!/bin/bash

case "$1" in 
    --raspberry)
        echo "Starting docker image for Raspberry pi"
        docker run -it --privileged -e DISPLAY  --network host \
        maze-raspberry-image
        ;;
    --jetson)
        echo "Starting docker image for Jetson"
        docker run -it --runtime nvidia --privileged -e DISPLAY  --network host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /usr/local/zed/resources:/usr/local/zed/resources \
        -v /dev:/dev \
        maze-jetson-image
        ;;
    *)
        echo "Legg til enten --raspberry eller --jetson"
        ;;
esac