#!/bin/bash         
xhost local:root
docker run  --net=host --name carla_container  --runtime=nvidia  --gpus 'all,"capabilities=graphics,utility,display,video,compute"' -e SDL_VIDEODRIVER=x11 -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix  -it  carlasim/carla:0.9.11 ./CarlaUE4.sh

