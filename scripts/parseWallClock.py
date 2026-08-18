import sys
from pathlib import Path

import numpy as np


def read_wallclock(path: Path) -> np.ndarray:
    with open(path) as f:
        return np.array([float(line) for line in f if line.strip()])


def summarize(path: Path):
    stamps = read_wallclock(path)
    if len(stamps) < 2:
        raise ValueError(f"{path} has fewer than 2 timestamps, can't compute deltas")

    elapsed = stamps - stamps[0]
    total = elapsed[-1]
    per_step = np.diff(stamps)

    if np.any(per_step < 0):
        print(f"Warning: {path} has a negative per-step delta -- system clock "
              f"jumped mid-run. Consider switching to steady_clock.", file=sys.stderr)

    return total, per_step


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: python parse_wallclock.py <wallclock_file>")

    path = Path(sys.argv[1])
    total, per_step = summarize(path)
    print(f"{path.name}: total = {total:.3f} s over {len(per_step) + 1} steps, "
          f"mean per-step = {per_step.mean() * 1000:.2f} ms, "
          f"max per-step = {per_step.max() * 1000:.2f} ms")
