#!/bin/bash
set -e
CONSTS_FILE="include/consts.h"
OUTDIR="output/threadSweep"

NS=(10000 100000 1000000)
SEEDS=(1 2)
THREADS=(1 2 4 8 16)
MASS=20
STEPS=$((50 + 100))

mkdir -p "$OUTDIR"

sed -i "s/constexpr double centMass = [0-9.]*/constexpr double centMass = $MASS/" "$CONSTS_FILE"
sed -i "s/constexpr int runSteps = [0-9.]*/constexpr int runSteps = $STEPS/" "$CONSTS_FILE"
sed -i "s/constexpr int NUM_THREADS = [0-9]*/constexpr int NUM_THREADS = 0/" "$CONSTS_FILE"

for n in "${NS[@]}"; do
    for seed in "${SEEDS[@]}"; do
        sed -i "s/constexpr int N = [0-9]*/constexpr int N = $n/" "$CONSTS_FILE"
        sed -i "s/constexpr int SEED = [-0-9.]*/constexpr int SEED = $seed/" "$CONSTS_FILE"
        cmake --build build
        for t in "${THREADS[@]}"; do
            echo "=== N=$n seed=$seed threads=$t ==="
            OMP_NUM_THREADS=$t ./build/runSim --headless
            mv "output/wallclockN${n}_s${seed}.txt" \
            "$OUTDIR/wallclockN${n}_s${seed}_t${t}.txt"
            mv "output/counterM${MASS}_s${seed}.txt" \
            "$OUTDIR/counterN${n}_M${MASS}_s${seed}_t${t}.txt"
        done
    done
done
echo "All runs complete."
