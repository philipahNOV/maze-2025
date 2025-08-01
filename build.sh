#!/bin/bash

case "$1" in 
    --raspberry)
        echo "Bygger docker image for Raspberry pi"
        docker build -f raspberrypi.dockerfile -t maze-raspberry-image .
        ;;
    --jetson)
        echo "Bygger docker image for Jetson"
        docker build -f jetson.dockerfile -t maze-jetson-image .
        ;;
    *)
        echo "Legg til enten --raspberry eller --jetson"
        ;;
esac
