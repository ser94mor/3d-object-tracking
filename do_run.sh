#!/usr/bin/env bash

set -x
set -e

readonly DIRNAME="$(realpath "$(dirname "${0}")")"

cd "${DIRNAME}"/cmake-build
./3D_object_tracking

