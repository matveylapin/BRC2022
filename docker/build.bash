#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

IMG_NAME="tolya_image"
BASE_IMG="ros:humble"

### DOCKER BUILD --------------------------------------------------------- #

printf "\n BUILDING DOCKER IMAGE: ${IMG_NAME}"
printf "\n                  FROM: ${BASE_IMG}\n\n"

docker build -t "${IMG_NAME}" \
             -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR \
            --network=host \
            --build-arg base_img=${BASE_IMG}
