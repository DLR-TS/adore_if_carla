SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

$(shell git submodule update --init --recursive --depth 1 ${ROOT_DIR}/external/*)

include adore_if_carla.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all 
all: build

.PHONY: run
run:
	bash run_1_carla.sh 

.PHONY: install_nvidia_docker2
install_nvidia_docker2:
	bash install_nvidia_docker2.sh 

.PHONY: build
build:
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	cd "${ROOT_DIR}"/adore_if_ros_msg && make
	cd "${ROOT_DIR}"/plotlablib && make
	cd "${ROOT_DIR}" && docker compose build
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${IMAGE_NAME}):/tmp/${PROJECT}/build ${PROJECT}
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${IMAGE_NAME}):/tmp/${PROJECT}/launch ${PROJECT}

.PHONY: clean 
clean: 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	docker rm $$(docker ps -a -q --filter "ancestor=${IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${IMAGE_NAME}) 2> /dev/null || true
