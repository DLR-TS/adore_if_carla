<!--
/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Matthias Nichting
 ********************************************************************************/
-->

# ADORe CARLA bridge
This package contains ROS2-Nodes to convert and exchange data between the native ROS2 interface of CARLA (version 0.10.0) and ADORe. It allows to control a vehicle with ADORe inside the CARLA simulation.

Note: The ADORe CARLA bridge is experimental.


## Prerequisites
The [system requirements](https://github.com/DLR-TS/adore/blob/master/documentation/technical_reference_manual/getting_started/system_requirements.md) and the [prerequisites](https://github.com/DLR-TS/adore/blob/master/documentation/technical_reference_manual/getting_started/prerequisites.md) of ADORe need to be fulfilled first. Additionally, the CARLA Unreal Engine 5 requires NVIDIA RTX driver release 550 or later and a NVIDIA RTX 3000 series GPU as a minimum with at least 16 Gb of VRAM is highly recommended (source: [CARLA quick start guide](https://carla-ue5.readthedocs.io/en/latest/start_quickstart/)). As the ADORe CARLA bridge builds upon the official docker image of CARLA, the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit) needs to be installed on the host system. 


## Getting started
In order to make the map that CARLA is using in the simple example available for adore, `make copy_map` can be run in this directory. Executing the command `docker compose up carla_bridge` starts two containers. In addition to a container in which CARLA is running, the Python script [carla_bridge_example.py](ros2.py) is executed in the carla_bridge container. The Python script spawns a vehicle in CARLA and connects it to ROS2. The conversion between the CARLA topics and the ADORe topics is carried out by the nodes of this package. For this purpose, the [carla_bridge_example.py](carla_bridge_example.py) file can be executed from the cli. In order to allow the communication between the ROS2 Jazzy nodes of ADORe and the native ROS2 communication interface of CARLA, the middleware settings need to be adjusted. This can be achieved by prefixing the launch command which then results in:
```bash
FASTRTPS_DEFAULT_PROFILES_FILE=./fastrtps-profile.xml RMW_FASTRTPS_USE_QOS_FROM_XML=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch carla_bridge_example.py
```


## Overview of contained nodes
The package currently contains the following two nodes.

### Control command conversion
This node converts the control command of ADORe containing the reference acceleration and steering angle and publishes the CarlaEgoVehicleControl message of the ros-carla-msgs package. 

### Sensor data conversion
This node converts the data of the GNSS and IMU sensors and publishes VehicleStateDynamic message used by ADORe as well as the transformations.


## Extensions
The bridge can easily be extended to exchange other data. For this, the [stack.json](stack.json) and the [ros2.py](ros2.py) files can be adjusted accordingly. Further information on this can be found in the documentation of the PythonAPI. In order to ensure compatibility between the ROS2 jazzy nodes of ADORe and the native ROS2 interface of CARLA, the fastrtps settings for communication must be adjusted as specified in the file [fastrtps-profile.xml](fastrtps-profile.xml).