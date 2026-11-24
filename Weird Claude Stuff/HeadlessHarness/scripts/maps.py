"""Render harness snapshots (lon/lat per vertex) as small equirectangular panels.
usage: python3 maps.py out.png fields snap1.csv [snap2.csv ...]
fields: comma list of thick,felsic,damage,stress,ach,block,age,speed,cf"""
import sys, numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.tri import Triangulation

out, fields, snaps = sys.argv[1], sys.argv[2].split(","), sys.argv[3:]
cmaps = {"thick": ("terrain", 5, 60), "felsic": ("cividis", 0, 1), "damage": ("inferno", 0, 1), "stress": ("magma", 0, 1.2),
         "ach": ("viridis", 0, 2), "block": ("tab20", None, None), "age": ("plasma_r", 0, 1500), "speed": ("viridis", 0, 2), "cf": ("cividis", 0, 1)}
fig, axes = plt.subplots(len(snaps), len(fields), figsize=(3.6 * len(fields), 1.95 * len(snaps)), squeeze=False)
for r, path in enumerate(snaps):
    d = pd.read_csv(path)
    tri = Triangulation(d.lon, d.lat)
    # drop triangles that wrap around the date line
    x = d.lon.values[tri.triangles]
    tri.set_mask((x.max(1) - x.min(1)) > 90)
    for c, f in enumerate(fields):
        ax = axes[r, c]
        cm, lo, hi = cmaps[f]
        v = d[f].values.copy()
        if f == "block":
            v = np.where(v < 0, np.nan, (v * 2654435761 % 977) / 977.0)
            lo, hi = 0, 1
        ax.tripcolor(tri, v, cmap=cm, vmin=lo, vmax=hi, shading="gouraud", rasterized=True)
        ax.set_xlim(-180, 180); ax.set_ylim(-90, 90); ax.set_xticks([]); ax.set_yticks([])
        if r == 0: ax.set_title(f, fontsize=9)
        if c == 0: ax.set_ylabel(path.split("_")[-1].replace(".csv", "") + " Myr", fontsize=8)
plt.tight_layout(pad=0.3)
plt.savefig(out, dpi=110)
print("wrote", out)
