SHELL:=/bin/bash

.DEFAULT_GOAL := all

PROJECT="adore_if_carla"
VERSION="latest"
IMAGE_NAME="${PROJECT}:${VERSION}"
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=


.PHONY: help
help:
	@awk 'BEGIN {FS = ":.*##"; printf "Usage: make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

.PHONY: all 
all: build

.PHONY: update_submodules
update_submodules:
	git submodule update --init --recursive	


.PHONY: run
run:
	bash run_1_carla.sh 

.PHONY: install_nvidia_docker2
install_nvidia_docker2:
	bash install_nvidia_docker2.sh 

.PHONY: build
build:
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	-patch -N external/ros-bridge/install_dependencies.sh ros-bridge_install_dependencies.patch
	rm -f external/ros-bridge/install_dependencies.sh.rej
	cd external/ros-bridge/docker && ./build.sh -r noetic
	cd "${ROOT_DIR}"/adore_if_ros_msg && make
	cd "${ROOT_DIR}" && \
        touch CATKIN_IGNORE && \
        docker build --network="host" -t ${IMAGE_NAME} . 
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${IMAGE_NAME}):/tmp/${PROJECT}/build ${PROJECT}
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${IMAGE_NAME}):/tmp/${PROJECT}/launch ${PROJECT}

.PHONY: clean 
clean: 
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	rm -rf ${ROOT_DIR}/${PROJECT}/launch
	docker rm $$(docker ps -a -q --filter "ancestor=${IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${IMAGE_NAME}) 2> /dev/null || true
