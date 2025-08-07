#!/bin/bash

cd /home/raspberrypi/Documents/maze-2025/pi/src

lxterminal -e "bash -c 'DISPLAY=:0 python3 main.py; exec bash'"
