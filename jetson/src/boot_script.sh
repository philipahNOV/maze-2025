#!/bin/bash

cd /home/student/Documents/maze-2025/jetson/src

gnome-terminal -- bash -c "/usr/bin/python3 main.py; exec bash"
gnome-terminal -- bash -c "/usr/bin/python3 recovery_listener.py; exec bash"
sudo /usr/sbin/nvpmodel -m 0
sudo /usr/bin/jetson_clocks
