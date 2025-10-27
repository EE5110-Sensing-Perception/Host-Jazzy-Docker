#!/bin/bash

# ... (variables and copyright lines) ...

docker_name="jazzy_ws"
docker_tag="latest"
CURRENT_DIR="$(pwd)"
CONTAINER_MOUNT_POINT="/host_code" 

docker run -it --rm \
  --name ${docker_name} \
  --network host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v "${CURRENT_DIR}:${CONTAINER_MOUNT_POINT}:rw" \
  ${docker_name}:${docker_tag}
