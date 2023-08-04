# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter adore_if_carla.mk, $(notdir $(MAKEFILE_LIST))), adore_if_carla.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_IF_CARLA_PROJECT=adore_if_carla

ADORE_IF_CARLA_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ADORE_IF_CARLA_SUBMODULES_PATH:=${ADORE_IF_CARLA_MAKEFILE_PATH}
else
    ADORE_IF_CARLA_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${ADORE_IF_CARLA_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodule update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${ADORE_IF_CARLA_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${ADORE_IF_CARLA_MAKEFILE_PATH}

ADORE_IF_CARLA_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

ADORE_IF_CARLA_IMAGE=${ADORE_IF_CARLA_PROJECT}:${ADORE_IF_CARLA_TAG}

ADORE_IF_CARLA_CMAKE_BUILD_PATH="${ADORE_IF_CARLA_PROJECT}/build"
ADORE_IF_CARLA_CMAKE_INSTALL_PATH="${ADORE_IF_CARLA_CMAKE_BUILD_PATH}/install"

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

include ${ADORE_IF_CARLA_SUBMODULES_PATH}/plotlablib/plotlablib.mk
include ${ADORE_IF_CARLA_SUBMODULES_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk

#ifeq ($(shell dpkg-query -W -f='${Status}' nvidia-docker2 2>/dev/null | grep -c "installed"),0)
#    $(error "ERROR: The apt package 'nvidia-docker2' needed by adore_if_carla is installed.")
#endif

.PHONY: build_adore_if_carla 
build_adore_if_carla: ## Build adore_if_carla
	cd "${ADORE_IF_CARLA_MAKEFILE_PATH}" && make build

.PHONY: clean_adore_if_carla
clean_adore_if_carla: ## Clean adore_if_carla build artifacts
	cd "${ADORE_IF_CARLA_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_if_carla
branch_adore_if_carla: ## Returns the current docker safe/sanitized branch for adore_if_carla
	@printf "%s\n" ${ADORE_IF_CARLA_TAG}

.PHONY: image_adore_if_carla
image_adore_if_carla: ## Returns the current docker image name for adore_if_carla
	@printf "%s\n" ${ADORE_IF_CARLA_IMAGE}

endif
