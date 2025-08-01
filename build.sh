#!/bin/bash

DOCKER_BUILDKIT=1 docker build --ssh default -t maze-jetson-image .
