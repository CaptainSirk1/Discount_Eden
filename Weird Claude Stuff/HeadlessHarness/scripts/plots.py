"""Result figures for TECTONICS_REVISION_2. Small multiples, one y-scale per panel, fixed categorical order."""
import sys, os, re, numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = "/home/claude/work/results"; OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/claude/work/deliver/docs/figures"
os.makedirs(OUT, exist_ok=True)
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
INK, MUTED, GRID = "#2b2b2a", "#6b6a64", "#e6e5e0"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                     "lines.linewidth": 2, "legend.frameon": False, "axes.titlesize": 9.5, "axes.titleweight": "bold"})
R2 = 6371.0 ** 2 / 1e9


def baseline(path):
    rows = []
    for l in open(path):
        if re.match(r"^\s+\d", l):
            p = l.split(); rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
    return pd.DataFrame(rows, columns=["t", "crust", "contVol", "contArea"])


def panel(ax, series, title, ylabel, ylim0=True):
    for k, (label, x, y) in enumerate(series):
        ax.plot(x / 1000, y, color=C[k], label=label)
    ax.set_title(title, loc="left"); ax.set_ylabel(ylabel); ax.set_xlabel("simulated time (Gyr)")
    if ylim0: ax.set_ylim(bottom=0)
    ax.legend(loc="best", fontsize=8)


# ---- 1. baseline vs revision, 4 Gyr, 5k vertices ----
fig, axs = plt.subplots(1, 3, figsize=(12, 3.3))
bs, bu = baseline(f"{R}/baseline_seeded.txt"), baseline(f"{R}/baseline_uniform.txt")
ns, nu = pd.read_csv(f"{R}/F_E_seeded.csv"), pd.read_csv(f"{R}/F_E_uniform.csv")
ns4, nu4 = ns[ns.t <= 4001], nu[nu.t <= 4001]
panel(axs[0], [("original, seeded", bs.t, bs.contVol), ("revised, seeded", ns4.t, ns4.contVol),
               ("original, uniform", bu.t, bu.contVol), ("revised, uniform", nu4.t, nu4.contVol)],
      "Continental volume (thickness > 26 km)", "10⁹ km³")
panel(axs[1], [("original, seeded", bs.t, bs.crust), ("revised, seeded", ns4.t, ns4.crust),
               ("original, uniform", bu.t, bu.crust), ("revised, uniform", nu4.t, nu4.crust)], "Total crust volume", "10⁹ km³")
panel(axs[2], [("revised, seeded", ns4.t, ns4.felsic), ("revised, uniform", nu4.t, nu4.felsic)],
      "Differentiated (felsic) volume — revised only", "10⁹ km³")
for l in axs[2].get_lines(): pass
axs[2].lines[0].set_color(C[1]); axs[2].lines[1].set_color(C[3]); axs[2].legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}/1_baseline_vs_revised_4gyr.png", dpi=130); plt.close(fig)

# ---- 2. 20 Gyr stress test ----
fig, axs = plt.subplots(2, 3, figsize=(12, 6.2))
for row, (d, name) in enumerate([(ns, "seeded"), (nu, "uniform")]):
    panel(axs[row, 0], [("continental (t > 26 km)", d.t, d.contVol), ("felsic material", d.t, d.felsic), ("all crust", d.t, d.crust)],
          f"{name}: volumes", "10⁹ km³")
    panel(axs[row, 1], [("continental area", d.t, d.contArea * 100), ("buoyant area", d.t, d.buoyArea * 100)], f"{name}: surface fractions", "% of surface")
    panel(axs[row, 2], [("mean speed", d.t, d.speed / 0.003), ("continental speed", d.t, d.contSpeed / 0.003)],
          f"{name}: activity", "× velocityReference")
fig.tight_layout(); fig.savefig(f"{OUT}/2_long_run_20gyr.png", dpi=130); plt.close(fig)

# ---- 3. cumulative ledger, seeded 20 Gyr ----
fig, axs = plt.subplots(1, 2, figsize=(11, 3.4))
d = ns
panel(axs[0], [("rift / ridge crust", d.t, d.rift), ("arc magmatism", d.t, d.magma), ("floor fill", d.t, d.floor)], "Ledger sources (cumulative)", "10⁹ km³")
panel(axs[1], [("subducted", d.t, d.subducted), ("delaminated", d.t, d.delam), ("erosion recycled", d.t, d.eroRec)], "Ledger sinks (cumulative)", "10⁹ km³")
fig.tight_layout(); fig.savefig(f"{OUT}/3_ledger_20gyr_seeded.png", dpi=130); plt.close(fig)

# ---- 4. seeds / timestep / resolution sensitivity ----
def load(p):
    return pd.read_csv(p) if os.path.exists(p) else None
groups = [("Test I: seeds (seeded continents)", [f"I_s{s}" for s in (1, 2, 3)], [f"seed {s}" for s in (1, 2, 3)]),
          ("Test I: seeds (uniform crust)", [f"I_u{s}" for s in (1, 2, 3)], [f"seed {s}" for s in (1, 2, 3)]),
          ("Test H: timestep / resolution", ["H_dt09", "H_dt18", "H_dt36", "H_20k"], ["dt 0.9 Myr", "dt 1.8 (auto)", "dt 3.6 Myr", "20k verts"])]
fig, axs = plt.subplots(2, 3, figsize=(12, 6.2))
for c, (title, files, labels) in enumerate(groups):
    s1, s2 = [], []
    for f, lab in zip(files, labels):
        d = load(f"{R}/{f}.csv")
        if d is None: continue
        s1.append((lab, d.t, d.felsic)); s2.append((lab, d.t, d.contArea * 100))
    panel(axs[0, c], s1, title + " — felsic", "10⁹ km³")
    panel(axs[1, c], s2, title + " — continental area", "% of surface")
fig.tight_layout(); fig.savefig(f"{OUT}/4_sensitivity.png", dpi=130); plt.close(fig)
print("figures in", OUT)
