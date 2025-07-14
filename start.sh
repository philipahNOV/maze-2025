#!/bin/bash

docker build -t maze2025-app .

docker run --rm -it \
  --runtime=nvidia \
  --network host \
  --device /dev/video0 \
  -v /usr/local/zed:/usr/local/zed \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v $(pwd):/app \
  maze2025-app