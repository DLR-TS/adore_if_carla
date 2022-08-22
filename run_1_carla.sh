#!/bin/bash         
xhost local:root
docker rm --force carla_container >/dev/null 2> /dev/null || true 
docker run\
            --rm \
            --privileged \
            --net=host \
            --name carla_container \
            --gpus all \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -it carlasim/carla:0.9.12 /bin/bash ./CarlaUE4.sh -RenderOffScreen

