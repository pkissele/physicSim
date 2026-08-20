import re
import sys
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

WALL_PATTERN = re.compile(r"wallclockN(\d+)_s(\d+)_t(\d+)\.txt")
COUNT_PATTERN = re.compile(r"counterN(\d+)_M(\d+)_s(\d+)_t(\d+)\.txt")
WARMUP_STEPS = 100

def steady_state_per_step(path: Path) -> float:
    stamps = np.array([float(line) for line in open(path) if line.strip()])
    if len(stamps) <= WARMUP_STEPS + 2:
        raise ValueError(f"{path.name}: too few steps to skip warmup")
    stamps = stamps[WARMUP_STEPS:]
    return (stamps[-1] - stamps[0]) / (len(stamps) - 1)

folder = Path(sys.argv[1])
by_nt = defaultdict(list)

for entry in sorted(folder.iterdir()):
    m = WALL_PATTERN.match(entry.name)
    if m is None:
        continue
    n, seed, t = int(m.group(1)), int(m.group(2)), int(m.group(3))
    by_nt[(n, t)].append(steady_state_per_step(entry))

Ns = sorted({n for n, _ in by_nt})
Ts = sorted({t for _, t in by_nt})

fig, (ax_sp, ax_eff) = plt.subplots(1, 2, figsize=(12, 5))
colors = ["#6a3fbf", "#c44fa0", "#3f8fbf", "#bf7a3f"]
print(f"{'N':>9} {'thr':>4} {'s/step':>10} {'speedup':>8} {'eff':>6}")

for i, n in enumerate(Ns):
    have = [t for t in Ts if (n, t) in by_nt]
    if 1 not in have:
        print(f"N={n}: no 1-thread baseline, skipping", file=sys.stderr)
        continue
    means = np.array([np.mean(by_nt[(n, t)]) for t in have])
    spread = np.array([np.ptp(by_nt[(n, t)]) / 2 for t in have])
    baseline = means[have.index(1)]
    speedup = baseline / means
    sp_err = speedup * (spread / means)
    eff = speedup / np.array(have)
    c = colors[i % len(colors)]
    ax_sp.errorbar(have, speedup, yerr=sp_err, fmt="o-", color=c, markersize=6, capsize=4, label=f"N = {n:,}")
    ax_eff.errorbar(have, eff, yerr=sp_err / np.array(have), fmt="o-", color=c, markersize=6, capsize=4, label=f"N = {n:,}")
    for t, m, s, e in zip(have, means, speedup, eff):
        print(f"{n:>9} {t:>4} {m:>10.4f} {s:>8.2f} {e:>6.2f}")

ideal = np.array(Ts, dtype=float)

ax_sp.plot(ideal, ideal, ":", color="gray", label="Ideal")
ax_sp.set_xscale("log", base=2)
ax_sp.set_yscale("log", base=2)
ax_sp.set_xticks(Ts)
ax_sp.set_xticklabels(Ts)
ax_sp.set_yticks(Ts)
ax_sp.set_yticklabels(Ts)
ax_sp.set_xlabel("Threads")
ax_sp.set_ylabel("Speedup vs. 1 thread")
ax_sp.set_title("Strong scaling")
ax_sp.legend()
ax_eff.axhline(1.0, ls=":", color="gray")
ax_eff.set_xscale("log", base=2)
ax_eff.set_xticks(Ts)
ax_eff.set_xticklabels(Ts)
ax_eff.set_ylim(0, 1.1)
ax_eff.set_xlabel("Threads")
ax_eff.set_ylabel("Parallel efficiency")
ax_eff.set_title("Efficiency")
ax_eff.legend()
plt.tight_layout()
plt.show()
