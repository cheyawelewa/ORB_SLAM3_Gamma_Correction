#!/usr/bin/env python3
"""
Generate one evaluation plot per EuRoC sequence.

Reads all JSON result files from the given directory, groups them by dataset,
and produces one PNG per sequence with all runs overlaid.

Usage:
    python plot_summary.py <results_dir> [--output_dir <plots_dir>]
"""

import argparse
import json
import os
import sys
from collections import defaultdict

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    import numpy as np
except ImportError:
    print("ERROR: matplotlib and numpy are required for plotting.")
    sys.exit(1)

SEQUENCE_ORDER = ["MH01", "MH02", "MH03", "MH04", "MH05",
                  "V101", "V102", "V103", "V201", "V202", "V203"]


def load_results(results_dir):
    """Load all JSON result files, grouped by dataset."""
    grouped = defaultdict(list)
    for fname in sorted(os.listdir(results_dir)):
        if not fname.endswith(".json"):
            continue
        fpath = os.path.join(results_dir, fname)
        with open(fpath) as f:
            data = json.load(f)
        grouped[data["dataset"]].append(data)
    return grouped


def plot_sequence(dataset, entries, output_dir):
    """
    One plot per sequence. Each entry is one run (frames or keyframes).
    Subplots: ATE over time | RPE translation | RPE rotation
    """
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(f"EuRoC {dataset} — All Runs", fontsize=14)

    # Assign a color per unique run label
    run_labels = sorted({e["run"] for e in entries})
    colors = cm.tab10(np.linspace(0, 0.9, len(run_labels)))
    color_map = {r: colors[i] for i, r in enumerate(run_labels)}

    # Track which labels have been added to the legend
    seen_labels = set()

    for entry in entries:
        run   = entry["run"]
        ttype = entry["type"]           # "frames" or "keyframes"
        color = color_map[run]
        ls    = "-" if ttype == "frames" else "--"
        rmse_ate   = entry["ate_rmse"]
        rmse_trans = entry["rpe_trans_rmse"]
        rmse_rot   = entry["rpe_rot_rmse"]
        legend_label = f"{run} ({ttype})"

        ate_errors   = entry["ate_errors"]
        trans_errors = entry["rpe_trans_errors"]
        rot_errors   = entry["rpe_rot_errors"]

        kw = dict(color=color, linestyle=ls, linewidth=0.9, alpha=0.85)

        # ATE
        axes[0].plot(ate_errors, label=legend_label if legend_label not in seen_labels else "_",
                     **kw)
        axes[0].axhline(rmse_ate, color=color, linestyle=":", linewidth=0.8, alpha=0.6)
        seen_labels.add(legend_label)

        # RPE translation
        axes[1].plot(trans_errors, **kw)
        axes[1].axhline(rmse_trans, color=color, linestyle=":", linewidth=0.8, alpha=0.6)

        # RPE rotation
        axes[2].plot(rot_errors, **kw)
        axes[2].axhline(rmse_rot, color=color, linestyle=":", linewidth=0.8, alpha=0.6)

    axes[0].set_title("ATE per Frame")
    axes[0].set_xlabel("Frame")
    axes[0].set_ylabel("Error (m)")
    axes[0].grid(True)
    axes[0].legend(fontsize=7, loc="upper left")

    axes[1].set_title("RPE Translation")
    axes[1].set_xlabel("Frame pair")
    axes[1].set_ylabel("Error (m)")
    axes[1].grid(True)

    axes[2].set_title("RPE Rotation")
    axes[2].set_xlabel("Frame pair")
    axes[2].set_ylabel("Error (deg)")
    axes[2].grid(True)

    plt.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    outfile = os.path.join(output_dir, f"{dataset}_evaluation.png")
    plt.savefig(outfile, dpi=150)
    plt.close()
    print(f"  Saved: {outfile}")


def write_summary_txt(grouped, datasets, output_dir):
    """Write all results to a single human-readable txt file."""
    outfile = os.path.join(output_dir, "evaluation_summary.txt")
    col = "{:<8} {:<40} {:<12} {:>12} {:>12} {:>12}"
    header = col.format("Dataset", "Run", "Type", "ATE RMSE(m)", "RPE T RMSE(m)", "RPE R RMSE(deg)")
    divider = "-" * len(header)

    with open(outfile, "w") as f:
        f.write("EuRoC Trajectory Evaluation Results\n")
        f.write("=" * len(header) + "\n\n")
        f.write(header + "\n")
        f.write(divider + "\n")
        for dataset in datasets:
            for entry in sorted(grouped[dataset], key=lambda e: (e["run"], e["type"])):
                f.write(col.format(
                    entry["dataset"],
                    entry["run"],
                    entry["type"],
                    f"{entry['ate_rmse']:.6f}",
                    f"{entry['rpe_trans_rmse']:.6f}",
                    f"{entry['rpe_rot_rmse']:.4f}",
                ) + "\n")
            f.write(divider + "\n")

    print(f"  Summary saved: {outfile}")


def main():
    parser = argparse.ArgumentParser(description="Plot per-sequence evaluation results")
    parser.add_argument("results_dir", help="Directory containing JSON result files")
    parser.add_argument("--output_dir", default=None,
                        help="Where to save plots (default: same as results_dir)")
    args = parser.parse_args()

    output_dir = args.output_dir or args.results_dir
    grouped = load_results(args.results_dir)

    if not grouped:
        print(f"No JSON result files found in: {args.results_dir}")
        sys.exit(1)

    # Process in canonical EuRoC order, then any extras
    datasets = [d for d in SEQUENCE_ORDER if d in grouped]
    datasets += [d for d in sorted(grouped) if d not in datasets]

    print(f"Generating plots for {len(datasets)} sequence(s)...")
    for dataset in datasets:
        plot_sequence(dataset, grouped[dataset], output_dir)

    write_summary_txt(grouped, datasets, output_dir)


if __name__ == "__main__":
    main()
