#!/bin/bash

REPOSITORY_NAME="displacement_flexible_linear_objects"

DOCKER_USER=ros
DOCKER_GID=1000
DOCKER_UID=1000

echo "======================="
echo " Building docker image "
echo " IMAGE_TAG:   ${REPOSITORY_NAME}"
echo " DOCKER_USER: ${DOCKER_USER}"
echo " DOCKER_GID:  ${DOCKER_GID}"
echo " DOCKER_UID:  ${DOCKER_UID}"

if [[ -z "${HE_APT_GPG}" ]]; then
  HE_APT_GPG="$(curl -L https://devlab.pages.ocado.tech/files/apt-internal-public.key)"
  export HE_APT_GPG
fi

docker build ${DOCKER_FILE_PATH} --build-arg USERNAME="${DOCKER_USER}"\
                                 --build-arg GID=${DOCKER_GID}\
                                 --build-arg UID=${DOCKER_UID}\
                                 --secret id=netrc,src="${HOME}/.netrc" \
                                 --secret id=HE_APT_GPG\
                                 --progress=plain \
                                 -f ./docker/Dockerfile \
                                 -t "${REPOSITORY_NAME}" .
