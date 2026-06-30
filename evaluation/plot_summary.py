#!/usr/bin/env python3
"""
Generate one evaluation plot per EuRoC sequence.

Reads all JSON result files from the given directory, groups them by dataset,
and produces one PNG per sequence with all runs overlaid.

Usage:
    python plot_summary.py <results_dir> [--output_dir <plots_dir>]
"""

import argparse
import csv
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

    run_labels = sorted({e["run"] for e in entries})
    colors = cm.tab10(np.linspace(0, 0.9, len(run_labels)))
    color_map = {r: colors[i] for i, r in enumerate(run_labels)}

    for entry in entries:
        run   = entry["run"]
        color = color_map[run]
        kw    = dict(color=color, linewidth=0.9, alpha=0.85)

        # ATE (from keyframes)
        if "ate_errors" in entry:
            axes[0].plot(entry["ate_errors"], label=run, **kw)
            axes[0].axhline(entry["ate_rmse"], color=color, linestyle=":", linewidth=0.8, alpha=0.6)

        # RPE translation (from frames)
        if "rpe_trans_errors" in entry:
            axes[1].plot(entry["rpe_trans_errors"], label=run, **kw)
            axes[1].axhline(entry["rpe_trans_rmse"], color=color, linestyle=":", linewidth=0.8, alpha=0.6)

        # RPE rotation (from frames)
        if "rpe_rot_errors" in entry:
            axes[2].plot(entry["rpe_rot_errors"], label=run, **kw)
            axes[2].axhline(entry["rpe_rot_rmse"], color=color, linestyle=":", linewidth=0.8, alpha=0.6)

    axes[0].set_title("ATE (keyframes)")
    axes[0].set_xlabel("Keyframe")
    axes[0].set_ylabel("Error (m)")
    axes[0].grid(True)
    axes[0].legend(fontsize=7, loc="upper left")

    axes[1].set_title("RPE Translation (frames)")
    axes[1].set_xlabel("Frame pair")
    axes[1].set_ylabel("Error (m)")
    axes[1].grid(True)

    axes[2].set_title("RPE Rotation (frames)")
    axes[2].set_xlabel("Frame pair")
    axes[2].set_ylabel("Error (deg)")
    axes[2].grid(True)

    plt.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    outfile = os.path.join(output_dir, f"{dataset}_evaluation.png")
    plt.savefig(outfile, dpi=150)
    plt.close()
    print(f"  Saved: {outfile}")


def write_csv_data(grouped, datasets, output_dir):
    """Save per-run error arrays into one CSV per run for external plotting tools."""
    csv_dir = os.path.join(output_dir, "csv")
    os.makedirs(csv_dir, exist_ok=True)

    for dataset in datasets:
        for entry in grouped[dataset]:
            run      = entry["run"]
            ate      = entry.get("ate_errors", [])
            rpe_t    = entry.get("rpe_trans_errors", [])
            rpe_r    = entry.get("rpe_rot_errors", [])
            n_rows   = max(len(ate), len(rpe_t))

            outfile  = os.path.join(csv_dir, f"{dataset}_{run}.csv")
            with open(outfile, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["keyframe_index", "ate_error_m",
                                 "frame_pair_index", "rpe_trans_error_m", "rpe_rot_error_deg"])
                for i in range(n_rows):
                    kf_idx  = i          if i < len(ate)   else ""
                    ate_val = ate[i]     if i < len(ate)   else ""
                    fp_idx  = i          if i < len(rpe_t) else ""
                    t_val   = rpe_t[i]   if i < len(rpe_t) else ""
                    r_val   = rpe_r[i]   if i < len(rpe_r) else ""
                    writer.writerow([kf_idx, ate_val, fp_idx, t_val, r_val])

    print(f"  CSV data saved to: {csv_dir}/")


def write_summary_txt(grouped, datasets, output_dir):
    """Write all results to a single human-readable txt file."""
    outfile = os.path.join(output_dir, "evaluation_summary.txt")
    col = "{:<8} {:<40} {:>14} {:>14} {:>16}"
    header = col.format("Dataset", "Run", "ATE RMSE(m)", "RPE T RMSE(m)", "RPE R RMSE(deg)")
    divider = "-" * len(header)

    with open(outfile, "w") as f:
        f.write("EuRoC Trajectory Evaluation Results\n")
        f.write("ATE from keyframes | RPE from frames\n")
        f.write("=" * len(header) + "\n\n")
        f.write(header + "\n")
        f.write(divider + "\n")
        for dataset in datasets:
            for entry in sorted(grouped[dataset], key=lambda e: e["run"]):
                ate   = f"{entry['ate_rmse']:.6f}"   if "ate_rmse"       in entry else "N/A"
                trans = f"{entry['rpe_trans_rmse']:.6f}" if "rpe_trans_rmse" in entry else "N/A"
                rot   = f"{entry['rpe_rot_rmse']:.4f}"   if "rpe_rot_rmse"   in entry else "N/A"
                f.write(col.format(entry["dataset"], entry["run"], ate, trans, rot) + "\n")
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

    write_csv_data(grouped, datasets, output_dir)
    write_summary_txt(grouped, datasets, output_dir)


if __name__ == "__main__":
    main()
