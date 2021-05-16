#!/usr/bin/env bash

set -x
set -e

readonly DIRNAME="$(realpath "$(dirname "${0}")")"

"${DIRNAME}"/do_build.sh
"${DIRNAME}"/do_run.sh

