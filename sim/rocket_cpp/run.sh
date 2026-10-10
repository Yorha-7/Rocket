#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

cmake -S . -B build
cmake --build build
./build/rocket_cpp
python3 ./scripts/trajectory.py
