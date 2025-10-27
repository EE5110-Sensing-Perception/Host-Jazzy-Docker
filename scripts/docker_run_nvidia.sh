#!/bin/bash

# ... (variables and copyright lines) ...

docker_name="nvidia_jazzy_ws"
docker_tag="latest"
CURRENT_DIR="$(pwd)"
CONTAINER_MOUNT_POINT="/ros2_ws" 

docker run -it --rm \
  --runtime=nvidia \
  --name ${docker_name} \
  --network host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility" \
  -v "${CURRENT_DIR}:${CONTAINER_MOUNT_POINT}:rw" \
  ${docker_name}:${docker_tag}
