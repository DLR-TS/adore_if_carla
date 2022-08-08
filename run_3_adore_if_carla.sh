#!/bin/bash
xhost local:root
docker rm --force adore_if_carla_container >/dev/null 2> /dev/null || true 
docker run\
            --rm \
            --net=host \
            --name adore_if_carla_container \
            --runtime=nvidia \
            --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
            -e SDL_VIDEODRIVER=x11 \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -it adore_if_carla:latest \
            /bin/bash -c 'source /tmp/adore_if_carla/build/catkin_generated/installspace/setup.bash; roslaunch /tmp/adore_if_carla/launch/demo014_adore_if_carla_part.launch'         
            