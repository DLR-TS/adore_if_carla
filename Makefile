SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

# auto clone mandatory submodules
$(shell git submodule update --init --recursive --depth 1 ${ROOT_DIR}/external/*)

include adore_if_carla.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all 
all: help

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
build: set_env start_apt_cacher_ng _build get_cache_statistics ## Build adore_if_carla 

.PHONY: _build
_build: build_adore_if_ros_msg build_plotlablib 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	cd "${ROOT_DIR}" && \
    docker compose build \
                         --build-arg CARLA_REPO=${CARLA_REPO} \
                         --build-arg CARLA_TAG=${CARLA_TAG} \
                         --build-arg PLOTLABLIB_TAG=${PLOTLABLIB_TAG} \
                         --build-arg ADORE_IF_ROS_MSG_TAG=${ADORE_IF_ROS_MSG_TAG}

	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/build ${PROJECT}
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/launch ${PROJECT}

.PHONY: clean 
clean: 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
