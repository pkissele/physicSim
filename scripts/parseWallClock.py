import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

FNAME_PATTERN = re.compile(r"wallclockN(\d+)_s(\d+)\.txt")


def run_total(path: Path) -> float:
    stamps = np.array([float(line) for line in open(path) if line.strip()])
    return stamps[-1] - stamps[0]


folder = Path(sys.argv[1])

by_n = defaultdict(list)
for entry in sorted(folder.iterdir()):
    m = FNAME_PATTERN.match(entry.name)
    if m is None:
        continue
    n, seed = int(m.group(1)), int(m.group(2))
    by_n[n].append(run_total(entry))

Ns = np.array(sorted(by_n))
means = np.array([np.mean(by_n[n]) for n in Ns])
stds = np.array([np.std(by_n[n], ddof=1) if len(by_n[n]) > 1 else 0.0 for n in Ns])

# fit power law
slope, intercept = np.polyfit(np.log(Ns), np.log(means), 1)

fig, ax = plt.subplots(figsize=(7, 5))
ax.errorbar(Ns, means, fmt="o", color="#6a3fbf", markersize=7, capsize=4, label="Measured")

fit_N = np.linspace(Ns.min(), Ns.max(), 200)
ax.plot(fit_N, np.exp(intercept) * fit_N ** slope, "--", color="#c44fa0", label=f"exponent = {slope:.2f}")

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("N")
ax.set_ylabel("Wall-clock time (s)")
ax.set_title("Runtime vs N")
ax.legend()
plt.tight_layout()
plt.show()

print(f"exponent = {slope:.3f}")
