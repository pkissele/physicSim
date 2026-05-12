import os
import sys
import json
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.colors import LinearSegmentedColormap
from scipy.optimize import curve_fit
import numpy as np
from scipy.optimize import curve_fit, minimize_scalar


def read_folder(folder_path: str) -> dict[str, str]:
    data = {}

    for entry in os.scandir(folder_path):
        if not entry.is_file():
            continue

        key = int(os.path.splitext(entry.name)[0].split("M")[1])
        print(key)

        with open(entry.path, "r", encoding="utf-8") as f:
            nums = [float(n) for n in f.read().splitlines()]
            data[key] = list(zip(nums[::2], nums[1::2]))

    return data



folder = sys.argv[1] if len(sys.argv) > 1 else "."
result = read_folder(folder)
result = dict(sorted(result.items()))

plt.style.use("seaborn-v0_8-whitegrid")
mpl.rcParams.update({
    "axes.facecolor":    "#f7f6fc",
    "figure.facecolor":  "#ffffff",
    "axes.edgecolor":    "#3a2d6b",
    "axes.linewidth":    1.4,
    "axes.labelcolor":   "#1e1a3f",
    "axes.labelweight":  "bold",
    "axes.labelsize":    13,
    "axes.titlesize":    14,
    "axes.titleweight":  "bold",
    "axes.titlecolor":   "#1e1a3f",
    "grid.color":        "#c8c4e8",
    "grid.linewidth":    0.6,
    "grid.linestyle":    "--",
    "xtick.color":       "#3a2d6b",
    "ytick.color":       "#3a2d6b",
    "xtick.labelsize":   11,
    "ytick.labelsize":   11,
    "legend.framealpha": 0.9,
})

def truncate_cmap(cmap, min_val=0.0, max_val=1.0, n=256):
    new_cmap = LinearSegmentedColormap.from_list(
        f'trunc({cmap.name},{min_val:.2f},{max_val:.2f})',
        cmap(np.linspace(min_val, max_val, n))
    )
    return new_cmap


cmap = truncate_cmap(cm.plasma, 0.10, 0.88)
fig, ax = plt.subplots(figsize=(9, 5.5))

keys = list(result.keys())
norm = mcolors.Normalize(vmin=min(keys), vmax=max(keys))

for key in result.keys():
    curData = result[key][100:]
    x = [t[0] - 0.033 * 100 for t in curData]
    y = [t[1] for t in curData]
    ax.plot(x, y, linewidth=2.5, color=cmap(norm(key)), alpha=0.9)

cb = fig.colorbar(
    cm.ScalarMappable(norm=norm, cmap=cmap),
    ax=ax, label="PBH Mass ($M_\odot$)", pad=0.02
)
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
plt.show()

def model_func(x, a, k, c):
    return a * (1 - np.exp(-k * (x-5)))

def model_func2(x, a, k, c):
    return a * np.log(k*(x-c))

def gompertz(t, A, b, c):
    return A * np.exp(-b * np.exp(-c * t))

def delayed_power(t, A, t0, n):
    return A * np.maximum(0.0, t - t0) ** n

finalA = []
finalM_log = []
A_errs = []
masses_full = []

for m in result.keys():
    curData = result[m][100:]
    x = [t[0] for t in curData]
    y = [t[1] for t in curData]

    initial_guess = [1, 0.2, 0]

    popt, pcov = curve_fit(
        delayed_power, x, y,
        p0=initial_guess,
        bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]),  # a and k must be positive
        maxfev=10000
    )
    a_fit, t0_fit, n_fit = popt
    print(f"Fitted Parameters for mass {m}: A={a_fit:.3f}, t0={t0_fit:.3f}, n={n_fit:.3f}")
    sigma_A = np.sqrt(np.diag(pcov))[0]   # 1σ uncertainty on A only
    A_errs.append(sigma_A)
    finalA.append(a_fit)
    masses_full.append(m)
    finalM_log.append(m)

finalM_log = np.log(finalM_log)
masses_full = np.array(masses_full)
A_errs = np.array(A_errs)
A_arr = np.array(finalA)

log_M  = np.log(masses_full)
w = 1.0 / A_errs

coeffs, cov = np.polyfit(log_M, A_arr, 1, w=w, cov=True)
alpha, C = coeffs
sigma_alpha = np.sqrt(cov[0, 0])


A_pred  = alpha * log_M + C
ss_res  = np.sum((A_arr - A_pred) ** 2)
ss_tot  = np.sum((A_arr - A_arr.mean()) ** 2)
r2      = 1 - ss_res / ss_tot
print(f"A = α·ln(M_\\odot) + C → α = {alpha:.3f} ± {sigma_alpha:.3f},  C = {C:.3f},  R² = {r2:.3f}")

fig, ax = plt.subplots(figsize=(7, 4.5))

m_range   = np.linspace(masses_full.min(), masses_full.max(), 300)
fit_curve = alpha * np.log(m_range) + C

for m, a, a_hat in zip(masses_full, A_arr, A_pred):
    ax.plot([m, m], [a, a_hat], color="#a08ee0", linewidth=1.0,
            linestyle="--", zorder=2)

ax.plot(m_range, fit_curve, "--", color="#c44fa0", linewidth=1.8,
        label=f"$A = {alpha:.2f}\\,\\ln M_\\odot + {C:.2f}$"
              f"\n$R^2 = {r2:.3f},\\;\\sigma_\\alpha = {sigma_alpha:.2f}$")

ax.scatter(masses_full, A_arr,
           color="#6a3fbf", s=60, zorder=3,
           edgecolors="#1e1a3f", linewidths=0.6, label="Fitted $A$")

ax.set_xscale("log")
ax.set_xlabel("Log PBH Mass ($M_\\odot$)")
ax.set_ylabel("Amplitude  $A$")
ax.set_title("Star Formation Amplitude vs PBH Mass", color="#1e1a3f")
ax.legend(fontsize=10)
ax.spines[["top", "right"]].set_visible(False)

plt.tight_layout()
plt.show()

