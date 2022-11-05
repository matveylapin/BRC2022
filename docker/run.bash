#!/bin/bash

# Written by Nikolay Dema <ndema2301@gmail.com>, September 2022

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true

IMG_NAME="tolya_image"

mkdir -p ${REPO_ROOT}/.vscode-server/bin || true
mkdir -p ${REPO_ROOT}/.vscode-server/data || true


### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm \
            -e "DISPLAY=${DISPLAY}" \
            -e "QT_X11_NO_MITSHM=1" \
            -e XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v ${ROOT_DIR}/slave_ws:/slave_ws \
            -v ~/.ssh:/home/robot/.ssh \
            -v /dev:/dev \
            -v ~/.Xauthority:/home/robot/.Xauthority \
            -v ${REPO_ROOT}/.vscode-server/bin:/home/robot/.vscode-server/bin \
            -v ${REPO_ROOT}/.vscode-server/data:/home/robot/.vscode-server/data \
            --mount 'source=vscode-server,target=/home/robot/.vscode-server/extensions,type=volume' \
            --net=host \
            --privileged \
            --name "tolya_container" ${IMG_NAME} \
            > /dev/null
