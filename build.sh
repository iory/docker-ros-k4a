#!/bin/bash
docker build \
       --network=host \
       -t iory/k4a:melodic \
       .
