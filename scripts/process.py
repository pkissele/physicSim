import os
import re
import sys
from collections import defaultdict

import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from scipy.optimize import curve_fit

BURN_IN_FRAMES = 100

DT_PER_FRAME = 0.033
PLOT_TIME_OFFSET = DT_PER_FRAME * BURN_IN_FRAMES

MODEL_T0 = 2.5

FIT_EXCLUDE_MASSES: set[int] = set()

FNAME_PATTERN = re.compile(r"M(\d+)_s(\d+)")


def read_folder(folder_path: str) -> dict[int, dict[int, list[tuple[float, float]]]]:
    # Read M<mass>_s<seed>.txt files into {mass: {seed: [(t, n), ...]}}.
    data: dict[int, dict[int, list[tuple[float, float]]]] = defaultdict(dict)

    for entry in os.scandir(folder_path):
        if not entry.is_file():
            continue
        m = FNAME_PATTERN.search(entry.name)
        if m is None:
            print(f"Skipping unrecognized file: {entry.name}", file=sys.stderr)
            continue
        mass, seed = int(m.group(1)), int(m.group(2))

        with open(entry.path, "r", encoding="utf-8") as f:
            nums = [float(value) for value in f.read().splitlines()]
        if len(nums) % 2 != 0:
            print(f"Warning: odd value count in {entry.name}, dropping last value", file=sys.stderr)
            nums = nums[:-1]

        data[mass][seed] = list(zip(nums[::2], nums[1::2]))

    return dict(sorted(data.items()))


def truncate_cmap(cmap, min_val=0.0, max_val=1.0, n=256):
    colors = cmap(np.linspace(min_val, max_val, n))
    return LinearSegmentedColormap.from_list(f"trunc({cmap.name},{min_val:.2f},{max_val:.2f})", colors)


# Model

def exp_sat(x, a, k):
    return a * (1 - np.exp(-k * (x - MODEL_T0)))


def exp_sat_free_t0(x, a, k, t0):
    return a * (1 - np.exp(-k * np.maximum(x - t0, 0.0)))


# Plot style
plt.style.use("seaborn-v0_8-whitegrid")
mpl.rcParams.update({
    "axes.linewidth": 1.4,
    "axes.labelweight": "bold",
    "axes.labelsize": 13,
    "axes.titlesize": 14,
    "axes.titleweight": "bold",
    "grid.linewidth": 0.6,
    "grid.linestyle": "--",
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "legend.framealpha": 0.9,
})


# Main
def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    result = read_folder(folder)

    if not result:
        sys.exit(f"No M<mass>_s<seed>.txt files found in {folder!r}")

    # Plot 1
    cmap = truncate_cmap(cm.plasma, 0.10, 0.88)
    fig, ax = plt.subplots(figsize=(9, 5.5))

    masses_all = list(result.keys())
    norm = mcolors.Normalize(vmin=min(masses_all), vmax=max(masses_all))

    for mass, seeds in result.items():
        color = cmap(norm(mass))
        series = [np.array(runs[BURN_IN_FRAMES:]) for runs in seeds.values()]
        lengths = {len(s) for s in series}
        if len(lengths) != 1:
            print(f"Warning: mass {mass} has mismatched series lengths {lengths}, "
                  f"truncating to shortest", file=sys.stderr)
            n_min = min(lengths)
            series = [s[:n_min] for s in series]

        x = series[0][:, 0] - PLOT_TIME_OFFSET
        ys = np.stack([s[:, 1] for s in series])  # (n_seeds, n_frames)

        y_mean = ys.mean(axis=0)
        ax.plot(x, y_mean, linewidth=2.5, color=color, alpha=0.9)

        if ys.shape[0] > 1:
            ax.fill_between(x, ys.min(axis=0), ys.max(axis=0), color=color, alpha=0.15, linewidth=0)

    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    cb = fig.colorbar(sm, ax=ax, label="PBH Mass", pad=0.02)
    cb.outline.set_edgecolor("#3a2d6b")
    cb.outline.set_linewidth(1.0)
    cb.ax.yaxis.label.set_color("#1e1a3f")
    cb.ax.tick_params(colors="#3a2d6b")

    ax.set_title("Number of Stars Formed vs Time for Varying PBH Mass")
    ax.set_xlabel("Time")
    ax.set_ylabel("Number of Stars Formed")
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color("#3a2d6b")

    plt.tight_layout()
    fig.savefig("fig_nums.png", dpi=200)
    plt.show()

    # Per-realization fits
    masses, lnA_mean, lnA_err, n_seeds_used = [], [], [], []

    for mass, seeds in result.items():
        if mass in FIT_EXCLUDE_MASSES:
            continue

        lnA_i, sigrel_i = [], []

        for seed, series in seeds.items():
            arr = np.array(series[BURN_IN_FRAMES:])
            x, y = arr[:, 0], arr[:, 1]

            try:
                p0 = [max(y.max(), 1.0), 0.5]
                popt, pcov = curve_fit(exp_sat, x, y, p0=p0, bounds=([0, 0], [np.inf, np.inf]), maxfev=10000)
            except RuntimeError as e:
                print(f"Fit failed for M={mass} seed={seed}: {e}", file=sys.stderr)
                continue

            a_fit, k_fit = popt
            perr = np.sqrt(np.diag(pcov))

            if not np.all(np.isfinite(perr)):
                print(f"Warning: non-finite covariance for M={mass} seed={seed}, "
                      f"dropping this realization's fit uncertainty", file=sys.stderr)
                continue

            print(f"M={mass:3d} seed={seed:2d}  A={a_fit:.3f}  k={k_fit:.3f}  "
                  f"sigma_A={perr[0]:.3f}")

            lnA_i.append(np.log(a_fit))
            sigrel_i.append(perr[0] / a_fit)  # sigma_lnA = sigma_A / A

        if len(lnA_i) == 0:
            print(f"No usable fits for mass {mass}, skipping", file=sys.stderr)
            continue

        lnA_i = np.array(lnA_i)
        sigrel_i = np.array(sigrel_i)
        n = len(lnA_i)

        sem = lnA_i.std(ddof=1) / np.sqrt(n) if n > 1 else 0.0
        fit_unc = np.sqrt(np.sum(sigrel_i ** 2)) / n
        combined_err = np.hypot(sem, fit_unc)

        masses.append(mass)
        lnA_mean.append(lnA_i.mean())
        lnA_err.append(combined_err if combined_err > 0 else np.nan)
        n_seeds_used.append(n)

    masses = np.array(masses, dtype=float)
    lnA_mean = np.array(lnA_mean)
    lnA_err = np.array(lnA_err)
    n_seeds_used = np.array(n_seeds_used)

    if np.any(np.isnan(lnA_err)) or np.any(lnA_err == 0):
        print("Warning: some masses have zero/undefined combined error "
              "(likely single-seed with a degenerate fit); "
              "these will be down-weighted using the smallest nonzero error instead.",
              file=sys.stderr)
        floor = np.nanmin(lnA_err[lnA_err > 0]) if np.any(lnA_err > 0) else 1.0
        lnA_err = np.where((lnA_err == 0) | np.isnan(lnA_err), floor, lnA_err)

    # Weighted regression
    weights = 1.0 / lnA_err
    (slope, intercept), cov = np.polyfit(masses, lnA_mean, 1, w=weights, cov="unscaled")
    sigma_slope = np.sqrt(cov[0, 0])
    sigma_intercept = np.sqrt(cov[1, 1])

    lnA_pred = slope * masses + intercept
    resid = lnA_mean - lnA_pred

    ss_res = np.sum((resid / lnA_err) ** 2)  # weighted chi-square
    dof = len(masses) - 2
    chi2_red = ss_res / dof if dof > 0 else np.nan

    ss_res_unweighted = np.sum(resid ** 2)
    ss_tot = np.sum((lnA_mean - lnA_mean.mean()) ** 2)
    r2 = 1 - ss_res_unweighted / ss_tot

    n_total_seeds = int(n_seeds_used.sum())
    print(f"\nWeighted fit over {len(masses)} masses, {n_total_seeds} total realizations:")
    print(f"ln(A) = ({slope:.4f} +/- {sigma_slope:.4f}) * M + ({intercept:.3f} +/- {sigma_intercept:.3f})")
    print(f"R^2 = {r2:.4f}, reduced chi^2 = {chi2_red:.3f}")

    excluded = sorted(set(result.keys()) - set(masses.astype(int)) - FIT_EXCLUDE_MASSES)
    if FIT_EXCLUDE_MASSES:
        print(f"Excluded masses (FIT_EXCLUDE_MASSES): {sorted(FIT_EXCLUDE_MASSES)}")
    if excluded:
        print(f"Masses present in data but dropped due to fit failures: {excluded}")

    # Plot 2: ln(A) vs mass
    fig, ax = plt.subplots(figsize=(8, 5))

    m_range = np.linspace(masses.min(), masses.max(), 300)
    fit_line = slope * m_range + intercept

    fit_label = (f"$\\ln A = ({slope:.4f}\\pm{sigma_slope:.4f})\\,M + ({intercept:.3f}\\pm{sigma_intercept:.3f})$\n"
                 f"$R^2 = {r2:.3f},\\ \\chi^2_\\nu = {chi2_red:.2f}$")
    ax.plot(m_range, fit_line, "--", color="#c44fa0", linewidth=1.8, zorder=3, label=fit_label)
    ax.errorbar(masses, lnA_mean, yerr=lnA_err, fmt="o", color="#6a3fbf", markersize=7,
                capsize=4, zorder=4, markeredgecolor="#1e1a3f", markeredgewidth=0.6,
                label="Mean $\\ln(A)$ across realizations")

    ax.set_xlabel("PBH Mass")
    ax.set_ylabel("$\\ln$(Amplitude  $A$)")
    ax.set_title("Log Star Formation Amplitude vs PBH Mass", color="#1e1a3f")
    ax.legend(fontsize=10)
    ax.spines[["top", "right"]].set_visible(False)

    plt.tight_layout()
    fig.savefig("fig_linreg.png", dpi=200)
    plt.show()


if __name__ == "__main__":
    main()
