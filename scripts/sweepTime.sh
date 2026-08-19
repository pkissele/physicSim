#!/bin/bash
set -e

CONSTS_FILE="include/consts.h"
NS=(10000 50000 100000 250000 500000 750000 1000000)
SEEDS=(1 2 3)
MASS=20
STEPS=$((500 + 100))
THREADS=1

sed -i "s/constexpr double centMass = [0-9.]*/constexpr double centMass = $MASS/" "$CONSTS_FILE"
sed -i "s/constexpr int runSteps = [0-9.]*/constexpr int runSteps = $STEPS/" "$CONSTS_FILE"

for n in "${NS[@]}"; do
    sed -i "s/constexpr int N = [0-9]*/constexpr int N = $n/" "$CONSTS_FILE"

    for seed in "${SEEDS[@]}"; do
        sed -i "s/constexpr int SEED = [-0-9.]*/constexpr int SEED = $seed/" "$CONSTS_FILE"
        cmake --build build

        OMP_NUM_THREADS=$THREADS ./build/runSim --headless

        echo "Done N=$n seed=$seed"
    done
done

echo "All runs complete."
