#!/bin/bash
docker build \
       --network=host \
       -t iory/docker-ros-k4a:latest \
       .
