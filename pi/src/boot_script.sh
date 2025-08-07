#!/bin/bash

cd /home/raspberrypi/Documents/maze-2025/pi/src

lxterminal -e "bash -c '\
  source /home/raspberrypi/.venvs/maze-2025-venv/bin/activate && \
  DISPLAY=:0 python main.py; \
  exec bash'"
