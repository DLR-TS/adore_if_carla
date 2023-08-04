SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")


MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

SUBMODULES_PATH?=${ROOT_DIR}

ROS_BRIDGE_PATH=${ROOT_DIR}/external/ros-bridge

include adore_if_carla.mk

.PHONY: init_ros-bridge_submodule
init_ros-bridge_submodule:
ifeq ($(wildcard $(ROS_BRIDGE_PATH)/*),)
  $(shell git submodule update --init --recursive --remote --depth 1 --jobs 4 --single-branch ${ROS_BRIDGE_PATH})
else
	@echo "ros-bridge submodule already initialized, skipping submodule init for sumo."
endif



.PHONY: all 
all: build

.PHONY: build
build: clean init_ros-bridge_submodule root_check docker_group_check build ## Build build adore_if_carla

.PHONY: set_env 
set_env:
	$(eval PROJECT := ${ADORE_IF_CARLA_PROJECT}) 
	$(eval TAG := ${ADORE_IF_CARLA_TAG})

.PHONY: run
run:
	bash run_1_carla.sh 

.PHONY: install_nvidia_docker2
install_nvidia_docker2:
	bash install_nvidia_docker2.sh 

.PHONY: build
build: set_env build_adore_if_ros_msg build_plotlablib
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	cd "${ROOT_DIR}" && docker compose build
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build ${PROJECT}
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/launch ${PROJECT}

.PHONY: clean
clean: set_env ## Clean adore_if_carla build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
