#!/usr/bin/env bash
rm -r build
set -euo pipefail

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
cd scripts
../build/bin/demo && python3 plot.py 


