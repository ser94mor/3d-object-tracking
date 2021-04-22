#!/usr/bin/env bash

set -x
set -e

readonly REBUILD_DOCKER_IMAGE="${REBUILD_DOCKER_IMAGE:-false}"
readonly DIRNAME="$(realpath "$(dirname "${0}")")"

if ${REBUILD_DOCKER_IMAGE}; then
    docker rmi --force 3d-object-tracking:latest
    docker build --tag 3d-object-tracking:latest --file "${DIRNAME}"/docker/Dockerfile "${DIRNAME}"
else
    if [[ -z  "$(docker images --quiet --filter reference=3d-object-tracking:latest)" ]]; then
        cat "${DIRNAME}"/docker/docker-image.tar.xz* > "${DIRNAME}"/docker/docker-image.tar.xz
        docker load --input "${DIRNAME}"/docker/docker-image.tar.xz
    fi
fi

docker run --rm --tty --interactive --volume "${DIRNAME}":/workspace 3d-object-tracking:latest \
    bash -c 'mkdir --parents cmake-build; \
             cd cmake-build; \
	     cmake ..; \
	     make -j$(nproc --ignore 1); \
	     cd ../; \
	     chmod -R a+rw cmake-build/'

