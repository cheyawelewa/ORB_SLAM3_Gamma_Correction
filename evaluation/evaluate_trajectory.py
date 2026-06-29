#!/usr/bin/env python3
"""
Trajectory evaluation script: computes ATE and RPE for SLAM/VO systems.

Input file format (TUM): one pose per line
    timestamp tx ty tz qx qy qz qw

Usage:
    python evaluate_trajectory.py groundtruth.txt --frames frames.txt --keyframes keyframes.txt
    python evaluate_trajectory.py groundtruth.txt --frames frames.txt --rpe_delta 5 --save_plots
"""

import numpy as np
import argparse
import sys
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_trajectory(filepath):
    """Read a TUM-format trajectory file -> {timestamp: 4x4 np.array}."""
    poses = {}
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])

            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            poses[ts] = T
    return poses


# ---------------------------------------------------------------------------
# Timestamp association
# ---------------------------------------------------------------------------

def associate(est_poses, gt_poses, max_diff=0.02):
    """
    Pair each estimated timestamp with the nearest ground-truth timestamp.
    Returns list of (ts_est, ts_gt) tuples, sorted by ts_est.
    """
    gt_ts = sorted(gt_poses.keys())
    pairs = []
    for ts_e in sorted(est_poses.keys()):
        diffs = [abs(ts_e - ts_g) for ts_g in gt_ts]
        idx = int(np.argmin(diffs))
        if diffs[idx] <= max_diff:
            pairs.append((ts_e, gt_ts[idx]))
    return pairs


# ---------------------------------------------------------------------------
# Alignment (Umeyama, no scale)
# ---------------------------------------------------------------------------

def umeyama_se3(src, dst):
    """
    Compute the optimal rigid-body (SE3) transform that maps src -> dst.
    src, dst: 3xN arrays.
    Returns a 4x4 transformation matrix.
    """
    assert src.shape == dst.shape and src.shape[0] == 3
    n = src.shape[1]

    mu_s = src.mean(axis=1, keepdims=True)
    mu_d = dst.mean(axis=1, keepdims=True)
    s_c = src - mu_s
    d_c = dst - mu_d

    cov = (d_c @ s_c.T) / n
    U, _, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    t = mu_d - R @ mu_s

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t[:, 0]
    return T


# ---------------------------------------------------------------------------
# ATE
# ---------------------------------------------------------------------------

def compute_ate(est_poses, gt_poses, pairs):
    """
    Absolute Trajectory Error.
    Aligns the estimated trajectory to ground truth via SE3 (Umeyama),
    then reports per-frame translational errors.
    """
    est_xyz = np.array([est_poses[e][:3, 3] for e, _ in pairs]).T  # 3xN
    gt_xyz = np.array([gt_poses[g][:3, 3] for _, g in pairs]).T

    T_align = umeyama_se3(est_xyz, gt_xyz)

    errors = []
    for ts_e, ts_g in pairs:
        aligned = T_align @ est_poses[ts_e]
        err = np.linalg.norm(aligned[:3, 3] - gt_poses[ts_g][:3, 3])
        errors.append(err)

    errors = np.array(errors)
    return {
        "rmse":   float(np.sqrt(np.mean(errors ** 2))),
        "mean":   float(errors.mean()),
        "median": float(np.median(errors)),
        "std":    float(errors.std()),
        "max":    float(errors.max()),
        "errors": errors,
        "T_align": T_align,
    }


# ---------------------------------------------------------------------------
# RPE
# ---------------------------------------------------------------------------

def compute_rpe(est_poses, gt_poses, pairs, delta=1):
    """
    Relative Pose Error.
    Computes the error in relative motions separated by `delta` frames.
    Reports translational (m) and rotational (deg) components.
    """
    trans_errors, rot_errors = [], []

    for i in range(len(pairs) - delta):
        ts_e_i, ts_g_i = pairs[i]
        ts_e_j, ts_g_j = pairs[i + delta]

        delta_est = np.linalg.inv(est_poses[ts_e_i]) @ est_poses[ts_e_j]
        delta_gt  = np.linalg.inv(gt_poses[ts_g_i])  @ gt_poses[ts_g_j]

        error = np.linalg.inv(delta_gt) @ delta_est

        trans_errors.append(np.linalg.norm(error[:3, 3]))

        cos_angle = np.clip((np.trace(error[:3, :3]) - 1) / 2, -1.0, 1.0)
        rot_errors.append(np.degrees(np.arccos(cos_angle)))

    trans_errors = np.array(trans_errors)
    rot_errors   = np.array(rot_errors)

    return {
        "trans_rmse":   float(np.sqrt(np.mean(trans_errors ** 2))),
        "trans_mean":   float(trans_errors.mean()),
        "trans_median": float(np.median(trans_errors)),
        "trans_std":    float(trans_errors.std()),
        "trans_max":    float(trans_errors.max()),
        "rot_rmse":     float(np.sqrt(np.mean(rot_errors ** 2))),
        "rot_mean":     float(rot_errors.mean()),
        "rot_median":   float(np.median(rot_errors)),
        "rot_std":      float(rot_errors.std()),
        "rot_max":      float(rot_errors.max()),
        "trans_errors": trans_errors,
        "rot_errors":   rot_errors,
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_ate(ate):
    print("\nATE (Absolute Trajectory Error) [m]:")
    print(f"  RMSE   : {ate['rmse']:.6f}")
    print(f"  Mean   : {ate['mean']:.6f}")
    print(f"  Median : {ate['median']:.6f}")
    print(f"  Std    : {ate['std']:.6f}")
    print(f"  Max    : {ate['max']:.6f}")


def print_rpe(rpe, delta):
    print(f"\nRPE (Relative Pose Error) [delta={delta} frame(s)]:")
    print(f"  Translation RMSE   : {rpe['trans_rmse']:.6f} m")
    print(f"  Translation Mean   : {rpe['trans_mean']:.6f} m")
    print(f"  Translation Median : {rpe['trans_median']:.6f} m")
    print(f"  Translation Std    : {rpe['trans_std']:.6f} m")
    print(f"  Translation Max    : {rpe['trans_max']:.6f} m")
    print(f"  Rotation RMSE      : {rpe['rot_rmse']:.4f} deg")
    print(f"  Rotation Mean      : {rpe['rot_mean']:.4f} deg")
    print(f"  Rotation Median    : {rpe['rot_median']:.4f} deg")
    print(f"  Rotation Std       : {rpe['rot_std']:.4f} deg")
    print(f"  Rotation Max       : {rpe['rot_max']:.4f} deg")


# ---------------------------------------------------------------------------
# Optional plots
# ---------------------------------------------------------------------------

def save_plots(label, pairs, est_poses, gt_poses, ate, rpe, delta):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [warning] matplotlib not installed — skipping plots")
        return

    T_align = ate["T_align"]

    gt_xyz = np.array([gt_poses[g][:3, 3] for _, g in pairs])
    est_aligned = np.array([(T_align @ est_poses[e])[:3, 3] for e, _ in pairs])

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle(f"Trajectory Evaluation — {label}", fontsize=14)

    # Top-view trajectory
    ax = axes[0, 0]
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "b-", linewidth=1, label="Ground Truth")
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], "r-", linewidth=1, alpha=0.8, label="Estimated (aligned)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory — Top View")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True)

    # ATE over frames
    ax = axes[0, 1]
    ax.plot(ate["errors"], linewidth=0.8, color="steelblue")
    ax.axhline(ate["rmse"], color="r", linestyle="--", label=f"RMSE = {ate['rmse']:.4f} m")
    ax.set_xlabel("Frame")
    ax.set_ylabel("Error (m)")
    ax.set_title("ATE per Frame")
    ax.legend()
    ax.grid(True)

    # RPE translation
    ax = axes[1, 0]
    ax.plot(rpe["trans_errors"], linewidth=0.8, color="darkorange")
    ax.axhline(rpe["trans_rmse"], color="r", linestyle="--",
               label=f"RMSE = {rpe['trans_rmse']:.4f} m")
    ax.set_xlabel("Frame pair")
    ax.set_ylabel("Error (m)")
    ax.set_title(f"RPE Translation (delta={delta})")
    ax.legend()
    ax.grid(True)

    # RPE rotation
    ax = axes[1, 1]
    ax.plot(rpe["rot_errors"], linewidth=0.8, color="purple")
    ax.axhline(rpe["rot_rmse"], color="r", linestyle="--",
               label=f"RMSE = {rpe['rot_rmse']:.4f} deg")
    ax.set_xlabel("Frame pair")
    ax.set_ylabel("Error (deg)")
    ax.set_title(f"RPE Rotation (delta={delta})")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    outfile = f"evaluation_{label.lower().replace(' ', '_')}.png"
    plt.savefig(outfile, dpi=150)
    print(f"  Plots saved to: {outfile}")
    plt.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def evaluate(label, traj_file, gt_poses, max_diff, rpe_delta, save_plots_flag):
    print(f"\n{'='*60}")
    print(f"Evaluating {label}: {traj_file}")
    print("=" * 60)

    est_poses = read_trajectory(traj_file)
    print(f"  Estimated poses : {len(est_poses)}")
    print(f"  Ground truth    : {len(gt_poses)}")

    pairs = associate(est_poses, gt_poses, max_diff=max_diff)
    print(f"  Associated pairs: {len(pairs)}")

    if len(pairs) < 3:
        print("  ERROR: too few associations — check timestamps or --max_diff")
        return

    ate = compute_ate(est_poses, gt_poses, pairs)
    rpe = compute_rpe(est_poses, gt_poses, pairs, delta=rpe_delta)

    print_ate(ate)
    print_rpe(rpe, rpe_delta)

    if save_plots_flag:
        save_plots(label, pairs, est_poses, gt_poses, ate, rpe, rpe_delta)


def main():
    parser = argparse.ArgumentParser(
        description="Compute ATE and RPE for SLAM trajectory evaluation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("groundtruth", help="Ground truth file (TUM format)")
    parser.add_argument("--frames",    help="Estimated frame trajectory (TUM format)")
    parser.add_argument("--keyframes", help="Estimated keyframe trajectory (TUM format)")
    parser.add_argument("--max_diff", type=float, default=0.02,
                        help="Max timestamp difference for association in seconds (default: 0.02)")
    parser.add_argument("--rpe_delta", type=int, default=1,
                        help="Frame step for RPE computation (default: 1)")
    parser.add_argument("--save_plots", action="store_true",
                        help="Save PNG plots of trajectory and errors")
    args = parser.parse_args()

    if not args.frames and not args.keyframes:
        parser.error("Provide at least one of --frames or --keyframes")

    print(f"Reading ground truth: {args.groundtruth}")
    gt_poses = read_trajectory(args.groundtruth)
    print(f"  Loaded {len(gt_poses)} poses")

    if args.frames:
        evaluate("Frames", args.frames, gt_poses,
                 args.max_diff, args.rpe_delta, args.save_plots)

    if args.keyframes:
        evaluate("Keyframes", args.keyframes, gt_poses,
                 args.max_diff, args.rpe_delta, args.save_plots)


if __name__ == "__main__":
    main()
