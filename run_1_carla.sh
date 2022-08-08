#!/bin/bash         
xhost local:root
docker rm --force carla_container >/dev/null 2> /dev/null || true 
docker run\
            --rm \
            --net=host \
            --name carla_container \
            --runtime=nvidia \
            --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
            -e SDL_VIDEODRIVER=x11 \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -it  carlasim/carla:0.9.11 ./CarlaUE4.sh

