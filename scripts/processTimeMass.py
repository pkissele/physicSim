import os
import sys
import json
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from scipy.optimize import curve_fit
import numpy as np


def read_folder(folder_path: str) -> dict[str, str]:
    data = {}

    for entry in os.scandir(folder_path):
        if not entry.is_file():
            continue

        key = int(os.path.splitext(entry.name)[0].split(".")[0].split("M")[1])

        with open(entry.path, "r", encoding="utf-8") as f:
            nums = [float(n) for n in f.read().splitlines()]
            data[key] = list(zip(nums[::2], nums[1::2]))

    return data



folder = sys.argv[1] if len(sys.argv) > 1 else "."
result = read_folder(folder)
result = dict(sorted(result.items()))

# mpl.rcParams["font.weight"] = "bold"
mpl.rcParams["axes.labelweight"] = "bold"
mpl.rcParams["axes.titleweight"] = "bold"
mpl.rcParams["axes.linewidth"] = 2.0

plt.style.use("dark_background")
mpl.rcParams["figure.facecolor"] = "#1e1e2e"  # dark purple-grey
mpl.rcParams["axes.facecolor"] = "#1e1e2e"
plt.figure(figsize=(8, 5))

keys = list(result.keys())
norm = mcolors.Normalize(vmin=min(keys), vmax=max(keys))
cmap = cm.plasma
# cmap = cm.inferno
# cmap = cm.turbo

for key in result.keys():
    if(key in {1, 15, 20}): continue
    curData = result[key][100:]

    x = [t[0]-0.033*100 for t in curData]
    y = [t[1] for t in curData]

    plt.plot(x, y, linewidth=2, color=cmap(norm(key)))

plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=plt.gca(), label="Mass")
plt.title("Number of Stars Formed vs Time for Varying PBH Mass")
plt.xlabel("Time")
plt.ylabel("Number of Stars Formed")
# plt.xlim(left=0)
# plt.ylim(bottom=0)
plt.tight_layout()
plt.margins(0.05)
plt.show()


def model_func(x, a, k, c):
    return a * (1 - np.exp(-k * (x-5)))


finalK = []
finalM = []
for m in result.keys():
    if(m in {1, 15, 20}): continue
    curData = result[m][100:]
    x = [t[0] for t in curData]
    y = [t[1] for t in curData]

    initial_guess = [max(y), 0.2, 5]

    # popt, pcov = curve_fit(model_func, x, y, p0=initial_guess)
    popt, pcov = curve_fit(
        model_func, x, y,
        p0=initial_guess,
        bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]),  # a and k must be positive
        maxfev=10000
    )
    a_fit, k_fit, c_fit = popt
    print(f"Fitted Parameters for mass {m}: a={a_fit:.3f}, k={k_fit:.3f}, c={c_fit:.3f}")
    finalK.append(np.log(k_fit))
    finalM.append(np.log(m))

plt.scatter(finalM, finalK)
plt.show()
