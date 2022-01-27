#!/bin/bash

docker run \
    --rm \
    -it \
    -v ~/repos/Xmo/source/xmo_packages:/root/dev_ws/src \
    -v ~/repos/Xmo/source/launch:/root/dev_ws/launch \
    -v ~/repos/Xmo/source/config:/root/dev_ws/config \
    -v ~/repos/Xmo/source/install:/root/dev_ws/install \
    --name xmo \
    -h xmo \
    --network="host" \
    --pid=host \
    --privileged \
    xmo:latest
