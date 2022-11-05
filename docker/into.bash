#!/bin/bash
xhost +local:docker
docker exec -it "hsl_2022_kobuki" /bin/bash
