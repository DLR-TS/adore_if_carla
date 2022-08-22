<!--
********************************************************************************
* Copyright (C) 2017-2022 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Matthias Nichting
*   Jan Lauermann 
********************************************************************************
-->
# ADORe Interface to CARLA using ROS and carla-ros-bridge

**Note: adore_if_carla is experimental**

nvidia-docker2 must be installed on the host system. Instructions can be found here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

During build, the Dockerfile of the carlasimulator/ros-bridge:0.9.12 is built and the carlasim/carla:0.9.12 docker image is pulled from dockerhub.
The run.sh starts containers of these two images as well as the also built adore_if_carla container.
