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
An interface package, which enables ADORe to control CARLA autonomous vehicles 
via carla-ros-bridge.

**Note: adore_if_carla is experimental**

## Prerequsits
The following tools must be installed and configured for your system:
- nvida-docker2
nvidia-docker2 must be installed on the host system. Instructions can be found 
here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

Alternatively, you can run the provided setup target to install nvidia-docker2 
on ubuntu:
```bash
make install_nvidia_docker2
```
- make
- docker

## Getting Started
1. Install nvida-docker2
2. Source the adore environment:
```bash
cd ../
source adore.env
```
Alternatively, you can directly provide the `SUBMODULES_PATH` such as the 
following: 
```bash
SUBMODULES_PATH="$(realpath ../)" make <target>
```

3. Build adore_if_carla with provided build target:
```bash
make build
```
During build, the Dockerfile of the carlasimulator/ros-bridge is built and the 
carlasim/carla:0.9.13 docker image is pulled from dockerhub. Grab a coffee, 
carla is >17GB.

4. Start the adore_if_carla docker context with the provided target:
```bash
make up
```

5. Run a scenario
- There is a demo scenario provided: [adore_scenarios/demo014_adore_if_carla.launch](https://github.com/DLR-TS/adore_scenarios/blob/master/demo014_adore_if_carla.launch). This demo can be run with: 
```bash
make run_demo_carla_scenario
```
or 
```bash
cd ..
make cli
cd adore_scenarios
roslaunch <desired scenario launch file>.launch
