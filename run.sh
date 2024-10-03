#!/bin/bash

REPOSITORY_NAME="displacement_flexible_linear_objects"


DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume=./configs:/ws/configs/ \
--volume=./src:/ws/src/ \
"

DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
"

DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

# Run the command
docker run -it --rm \
                --net=host \
                --ipc=host \
                --hostname="$(hostname)" \
                --privileged  \
                ${DOCKER_ARGS} \
                ${REPOSITORY_NAME} $@
