import os
import sys

import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from scipy import stats
from scipy.optimize import curve_fit


def read_folder(folder_path: str) -> dict[int, list[tuple[float, float]]]:
    data = {}

    for entry in os.scandir(folder_path):
        if not entry.is_file():
            continue

        key = int(os.path.splitext(entry.name)[0].split("M")[1])
        print(key)

        with open(entry.path, "r", encoding="utf-8") as f:
            nums = [float(value) for value in f.read().splitlines()]

        data[key] = list(zip(nums[::2], nums[1::2]))

    return data


def truncate_cmap(cmap, min_val=0.0, max_val=1.0, n=256):
    return LinearSegmentedColormap.from_list(
        f"trunc({cmap.name},{min_val:.2f},{max_val:.2f})",
        cmap(np.linspace(min_val, max_val, n))
    )


def model_func(x, a, k, c):
    return a * (1 - np.exp(-k * (x - 5)))


def model_func2(x, a, k, c):
    return a * np.log(k * (x - c))


def gompertz(t, A, b, c):
    return A * np.exp(-b * np.exp(-c * t))


def delayed_power(t, A, t0, n):
    return A * np.maximum(0.0, t - t0) ** n


def exp_sat(x, a, k, c):
    return a * (1 - np.exp(-k * (x - 2.5)))


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


folder = sys.argv[1] if len(sys.argv) > 1 else "."
result = dict(sorted(read_folder(folder).items()))

# Plot star formation history
cmap = truncate_cmap(cm.plasma, 0.10, 0.88)
fig, ax = plt.subplots(figsize=(9, 5.5))

keys = list(result.keys())
norm = mcolors.Normalize(vmin=min(keys), vmax=max(keys))

for key in result:
    cur_data = result[key][100:]
    x = [t[0] - 0.033 * 100 for t in cur_data]
    y = [t[1] for t in cur_data]
    ax.plot(x, y, linewidth=2.5, color=cmap(norm(key)), alpha=0.9)

cb = fig.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, label="PBH Mass", pad=0.02)
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


# Fit delayed-power model
final_A = []
final_M_log = []
A_errs = []
masses_full = []

for mass in result:
    cur_data = result[mass][100:]
    x = [t[0] for t in cur_data]
    y = [t[1] for t in cur_data]

    popt, pcov = curve_fit(
        delayed_power,
        x,
        y,
        p0=[1, 0.2, 0],
        bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]),
        maxfev=10000
    )

    A_fit, t0_fit, n_fit = popt
    print(f"Fitted Parameters for mass {mass}: A={A_fit:.3f}, t0={t0_fit:.3f}, n={n_fit:.3f}")

    sigma_A = np.sqrt(np.diag(pcov))[0]

    A_errs.append(sigma_A)
    final_A.append(A_fit)
    masses_full.append(mass)
    final_M_log.append(mass)


final_M_log = np.log(final_M_log)
masses_full = np.array(masses_full)
A_errs = np.array(A_errs)
A_arr = np.array(final_A)

log_M = np.log(masses_full)
w = 1.0 / A_errs

coeffs, cov = np.polyfit(log_M, A_arr, 1, w=w, cov=True)
alpha, C = coeffs
sigma_alpha = np.sqrt(cov[0, 0])

A_pred = alpha * log_M + C
ss_res = np.sum((A_arr - A_pred) ** 2)
ss_tot = np.sum((A_arr - A_arr.mean()) ** 2)
r2 = 1 - ss_res / ss_tot

print(f"A = α·ln(M_\\odot) + C → α = {alpha:.3f} ± {sigma_alpha:.3f}, C = {C:.3f}, R² = {r2:.3f}")


# Plot amplitude scaling
fig, ax = plt.subplots(figsize=(7, 4.5))

m_range = np.linspace(masses_full.min(), masses_full.max(), 300)
fit_curve = alpha * np.log(m_range) + C

for mass, A_value, A_hat in zip(masses_full, A_arr, A_pred):
    ax.plot([mass, mass], [A_value, A_hat], color="#a08ee0", linewidth=1.0, linestyle="--", zorder=2)

ax.plot(
    m_range,
    fit_curve,
    "--",
    color="#c44fa0",
    linewidth=1.8,
    label=f"$A = {alpha:.2f}\\,\\ln M_\\odot + {C:.2f}$\n$R^2 = {r2:.3f},\\;\\sigma_\\alpha = {sigma_alpha:.2f}$"
)

ax.errorbar(
    masses_full,
    A_arr,
    # yerr=A_errs,
    fmt="o",
    color="#6a3fbf",
    markersize=7,
    capsize=4,
    zorder=3,
    markeredgecolor="#1e1a3f",
    markeredgewidth=0.6,
    label="Fitted $A$"
)

ax.set_xscale("log")
ax.set_xlabel("Log PBH Mass ($M_\\odot$)")
ax.set_ylabel("Amplitude  $A$")
ax.set_title("Star Formation Amplitude vs PBH Mass", color="#1e1a3f")
ax.legend(fontsize=10)
ax.spines[["top", "right"]].set_visible(False)

plt.tight_layout()
plt.show()


# Fit exponential saturation model
# fit_exclude = {30}
fit_exclude = {}
# plot_exclude = {1, 15, 20}
plot_exclude = {}

final_M_sat = []
final_A_sat = []
final_A_sat_errs = []

for mass in result:
    if mass in fit_exclude:
        continue

    cur_data = result[mass][100:]
    x = np.array([t[0] for t in cur_data])
    y = np.array([t[1] for t in cur_data])

    popt, pcov = curve_fit(
        exp_sat,
        x,
        y,
        p0=[50, 0.5, 2.5],
        bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]),
        maxfev=10000
    )

    a_fit, k_fit, c_fit = popt
    print(f"M ={mass:3d}  a={a_fit:.3f}  k={k_fit:.3f}  c={c_fit:.3f}")

    final_M_sat.append(mass)
    final_A_sat.append(np.log(a_fit))

    a_err = np.sqrt(np.diag(pcov))[0]
    final_A_sat_errs.append(a_err / a_fit)

final_M_sat = np.array(final_M_sat)
final_A_sat = np.array(final_A_sat)

slope_s, intercept_s, r_val, p_val, _ = stats.linregress(final_M_sat, final_A_sat)
A_pred_sat = slope_s * final_M_sat + intercept_s

ss_res = np.sum((final_A_sat - A_pred_sat) ** 2)
ss_tot = np.sum((final_A_sat - final_A_sat.mean()) ** 2)
r2_sat = 1 - ss_res / ss_tot

print(f"\nln(A) = {slope_s:.4f}·M + {intercept_s:.3f}")
print(f"R = {r_val:.6f},  R² = {r2_sat:.4f},  p = {p_val:.2e}")


# Plot logarithmic amplitude scaling
fig, ax = plt.subplots(figsize=(8, 5))

m_range = np.linspace(final_M_sat.min(), final_M_sat.max(), 300)
fit_line = slope_s * m_range + intercept_s

for mass, A_value, A_hat in zip(final_M_sat, final_A_sat, A_pred_sat):
    ax.plot([mass, mass], [A_value, A_hat], color="#a08ee0", linewidth=1.0, linestyle="--", zorder=2)

ax.plot(
    m_range,
    fit_line,
    "--",
    color="#c44fa0",
    linewidth=1.8,
    zorder=3,
    label=f"$\\ln A = {slope_s:.4f}\\,M + {intercept_s:.3f}$\n$R = {r_val:.4f},\\quad p = {p_val:.2e}$"
)

ax.errorbar(
    final_M_sat,
    final_A_sat,
    # yerr=final_A_sat_errs,
    fmt="o",
    color="#6a3fbf",
    markersize=7,
    capsize=4,
    zorder=4,
    markeredgecolor="#1e1a3f",
    markeredgewidth=0.6,
    label="Fitted $\\ln(A)$"
)

ax.set_xlabel("PBH Mass")
ax.set_ylabel("$\\ln$(Amplitude  $A$)")
ax.set_title("Log Star Formation Amplitude vs PBH Mass", color="#1e1a3f")
ax.legend(fontsize=10)
ax.spines[["top", "right"]].set_visible(False)

plt.tight_layout()
plt.show()
