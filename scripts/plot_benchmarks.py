#!/usr/bin/env python3
"""
plot_benchmarks.py

Reads app/build/benchmarks/sweep.csv (produced by SweepBenchmarkTest) and
generates two charts under Images/:

  - time_vs_N.png : query time per structure as N grows (M held fixed)
  - time_vs_M.png : query time per structure as M grows (N held fixed)

That's it — one variable changes, time on Y, one line per structure.

Run:
    pip install matplotlib pandas
    python3 scripts/plot_benchmarks.py
"""
from pathlib import Path
import sys

import pandas as pd
import matplotlib.pyplot as plt

CSV = Path("app/build/benchmarks/sweep.csv")
OUT = Path("/Users/suba/Desktop/SCHOOL/YEAR3/P2/INTERNSHIP/COMPUTER_GRAPHICS/AccellerationStructures/Images")

# Consistent color per structure across both charts.
COLORS = {
    "BruteForce":  "#777777",
    "UniformGrid": "#1f77b4",
    "Octree":      "#2ca02c",
    "KDTree":      "#d62728",
    "BVH":         "#9467bd",
}


def line_plot(df: pd.DataFrame, x_col: str, fixed_col: str,
              title: str, xlabel: str, outfile: Path) -> None:
    # Hold the other axis at its largest value so curves are smoothest.
    fixed_val = df[fixed_col].max()
    sub = df[df[fixed_col] == fixed_val]

    fig, ax = plt.subplots(figsize=(8, 5))
    for name in COLORS:
        grp = sub[sub["structure"] == name].sort_values(x_col)
        if grp.empty:
            continue
        ax.plot(grp[x_col], grp["query_ms"], marker="o",
                label=name, color=COLORS[name])

    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel(xlabel + " (log)")
    ax.set_ylabel("Query time (ms, log)")
    ax.set_title(f"{title}  ({fixed_col}={fixed_val:,} fixed)")
    ax.grid(True, which="both", ls=":", alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outfile, dpi=140)
    plt.close(fig)


def main() -> None:
    if not CSV.exists():
        sys.exit(f"CSV not found: {CSV}\nRun the SweepBenchmarkTest first.")
    OUT.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(CSV)

    line_plot(df, x_col="N", fixed_col="M",
              title="Query time vs scene size N",
              xlabel="Scene size N (objects)",
              outfile=OUT / "time_vs_N.png")

    line_plot(df, x_col="M", fixed_col="N",
              title="Query time vs ray count M",
              xlabel="Ray count M",
              outfile=OUT / "time_vs_M.png")

    print(f"Charts written to {OUT}/")


if __name__ == "__main__":
    main()
