#!/bin/bash

docker run \
    --rm \
    -it \
    --name xmo \
    -h xmo \
    --network="host" \
    --pid=host \
    --privileged \
    xmo:latest
