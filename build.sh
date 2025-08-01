#!/bin/bash

case "$1" in 
    --raspberry)
        echo "Bygger docker image for Raspberry pi"
        docker build -t maze-raspberry-image .
        ;;
    --jetson)
        echo "Bygger docker image for Jetson"
        docker build -t maze-jetson-image .
        ;;
    *)
        echo "Legg til enten --raspberry eller --jetson"
        ;;
esac
