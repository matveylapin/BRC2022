#!/bin/bash

SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

IMG_NAME="nickodema/hsl_2022"
IMG_TAG=""

### Check for NVIDIA GPU

if [ -n "$(which nvidia-smi)" ] && [ -n "$(nvidia-smi)" ]; then
    IMG_TAG="nvidia"
else
    IMG_TAG="general"
fi

docker build -t hsl_2022:${IMG_TAG} -f ${SIM_ROOT}/docker/Dockerfile ${SIM_ROOT} \
                                  --network=host \
                                  --build-arg from=${IMG_NAME}:${IMG_TAG}
