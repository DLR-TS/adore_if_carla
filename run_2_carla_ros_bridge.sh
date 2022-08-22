#!/bin/bash    
xhost local:root
docker rm --force carla_ros_bridge_container >/dev/null 2> /dev/null || true      
docker run\
            --rm \
            --net=host \
            --name carla_ros_bridge_container \
            --runtime=nvidia \
            --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
            -e SDL_VIDEODRIVER=x11 \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -it  carla-ros-bridge:noetic \
            /bin/bash -c 'source /opt/carla-ros-bridge/install/setup.bash; roslaunch /opt/carla-ros-bridge/src/carla_ros_bridge/launch/carla_ros_bridge_with_example_ego_vehicle.launch'
