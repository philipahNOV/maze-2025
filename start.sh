#!/bin/bash



docker run -it --runtime nvidia --privileged -e DISPLAY  --network host\
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/local/zed/resources:/usr/local/zed/resources \
  -v /dev:/dev \
  maze-2025-jetson


#docker build -t maze2025-app .

#docker run --rm -it \
#  --runtime=nvidia \
#  --network host \
#  --device /dev/video0 \
#  -v /usr/local/zed:/usr/local/zed \
#  -v /tmp/argus_socket:/tmp/argus_socket \
#  -v $(pwd):/app \
#  maze2025-app