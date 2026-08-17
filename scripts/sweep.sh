#!/bin/bash
set -e
 
CONSTS_FILE="include/consts.h"
MASSES=(1 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80)
SEEDS=(1 2 3 4 5)
STEPS=$((1500 + 100))
OUT=runs
mkdir -p "$OUT"

git rev-parse HEAD > "$OUT/commit.txt"

for mass in "${MASSES[@]}"; do
    for seed in "${SEEDS[@]}"; do
        sed -i "s/constexpr double centMass = [0-9.]*/constexpr double centMass = $mass/" "$CONSTS_FILE"
        sed -i "s/constexpr int SEED = [-0-9.]*/constexpr int SEED = $seed/" "$CONSTS_FILE"
        sed -i "s/constexpr int runSteps = [0-9.]*/constexpr int runSteps = $STEPS/" "$CONSTS_FILE"
        cmake --build build
 
        ./build/runSim --headless
 
        echo "Done M=$mass seed=$seed"
    done
done
 
echo "All runs complete."
