
# *******************************************************************************
# * Copyright (C) 2017-2022 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Matthias Nichting
# ********************************************************************************

cmake_minimum_required(VERSION 2.8.3)
project(adore_if_carla)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION ${PROJECT_NAME})
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")

include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#include(Packing)

list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")

set(PROJECT_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/build/devel/include")


set(CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH};/tmp/${PROJECT_NAME}/${PROJECT_NAME}/build/install")
list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")

set(ADORE_IF_ROS_MSG_CMAKE_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../adore_if_ros_msg/adore_if_ros_msg/build/install")
list(APPEND CMAKE_PREFIX_PATH "${ADORE_IF_ROS_MSG_CMAKE_INSTALL_PATH}")

set(PLOTLABLIB_CMAKE_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../plotlablib/plotlablib/build/install")
list(APPEND CMAKE_PREFIX_PATH "${PLOTLABLIB_CMAKE_INSTALL_PATH}")


list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cppzmq/build/install/share/cmake/cppzmq")
#/share/cmake/cppzmq/cppzmqConfig.cmake
message(WARNING ${CMAKE_CURRENT_SOURCE_DIR})



#set(carla_msgs_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    adore_if_ros_msg
    carla_msgs
    roscpp
    ackermann_msgs
)

find_package(plotlablib CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(ZeroMQ CONFIG REQUIRED)

#generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
#    nav_msgs
#    geometry_msgs
#  )

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES adore_if_carla
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs ackermann_msgs
  CATKIN_DEPENDS adore_if_ros_msg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(vehiclestate2adore_node)
target_sources(vehiclestate2adore_node PRIVATE src/vehiclestate2adore.cpp)
target_include_directories(vehiclestate2adore_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(vehiclestate2adore_node PRIVATE ${catkin_LIBRARIES})

add_executable(objects2adore_node)
target_sources(objects2adore_node PRIVATE src/objects2adore.cpp)
target_include_directories(objects2adore_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(objects2adore_node PRIVATE ${catkin_LIBRARIES})

add_executable(clock2simtime_node)
target_sources(clock2simtime_node PRIVATE src/clock2simtime.cpp)
target_include_directories(clock2simtime_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(clock2simtime_node PRIVATE ${catkin_LIBRARIES})

add_executable(controlcommand2carla_node)
target_sources(controlcommand2carla_node PRIVATE src/controlcommand2carla.cpp)
target_include_directories(controlcommand2carla_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(controlcommand2carla_node PRIVATE ${catkin_LIBRARIES})

add_executable(ackermanncommand2carla_node)
target_sources(ackermanncommand2carla_node PRIVATE src/ackermanncommand2carla.cpp)
target_include_directories(ackermanncommand2carla_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(ackermanncommand2carla_node PRIVATE ${catkin_LIBRARIES})

add_executable(plot_longitudinal_control_info_node src/plot_longitudinal_control_info.cpp)
target_include_directories(plot_longitudinal_control_info_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_longitudinal_control_info_node
                        PRIVATE 
                          ${catkin_LIBRARIES}
                          plotlablib::plotlablib
)

install(
  TARGETS
  vehiclestate2adore_node
  objects2adore_node
  controlcommand2carla_node
  ackermanncommand2carla_node
  clock2simtime_node
  plot_longitudinal_control_info_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  FILES
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
