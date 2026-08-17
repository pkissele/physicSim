#!/bin/bash
set -e

CONSTS_FILE="include/consts.h"
MASSES=(1 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80)

for mass in "${MASSES[@]}"; do
    echo "=== Running with centMass = $mass ==="

    # Patch centMass in consts.h using sed
    sed -i "s/constexpr double centMass = [0-9.]*/constexpr double centMass = $mass/" "$CONSTS_FILE"

    cmake --build build

    # Run headlessly or normally — add --save or other flags as needed
    ./build/runSim --headless

    echo "Done with centMass = $mass"
done

echo "All runs complete."
