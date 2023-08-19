#!/usr/bin/env bash

function echoerr { echo "$@" >&2; exit 1;}
SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

check_distro() {
    distribution=$(awk -F= '/^NAME/{print $2}' /etc/os-release | tr -d '"')
    if [ "$distribution" != "Ubuntu" ]; then
        echo "Error: This script is intended for Ubuntu distributions only." >&2
        exit 1
    fi
}

check_distro

distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/experimental/$distribution/libnvidia-container.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list


sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
