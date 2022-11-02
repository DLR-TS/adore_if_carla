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
# adore_if_carla
An interface package, which enables ADORe to control CARLA autonomous vehicles via carla-ros-bridge.
**Note: adore_if_carla is experimental**

## Getting Started
nvidia-docker2 must be installed on the host system. Instructions can be found here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

Build by executing ```make``` in the project folder.
During build, the Dockerfile of the carlasimulator/ros-bridge is built and the carlasim/carla:0.9.13 docker image is pulled from dockerhub.

Start the demo by following these steps, (replace adore project folder path as necessary):
~~~bash
cd ~/adore/adore_if_carla
xterm -e "docker compose up carla"
xterm -e "xhost local:root;docker compose up carla-ros-bridge"
xterm -e "docker compose up adore_if_carla"
cd ~/adore/  
make run_test_scenarios demo014_adore_if_carla.launch
~~~