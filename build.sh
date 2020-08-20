#!/bin/bash
docker build \
       --network=host \
       -t k4a:melodic \
       .
