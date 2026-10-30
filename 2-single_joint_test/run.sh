#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
cmake -S . -B build
cmake --build build -j
exec ./build/mujoco_joint "$@"
