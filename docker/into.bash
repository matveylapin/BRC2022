#!/bin/bash
xhost +local:docker
docker exec -it "tolya_container" /bin/bash
