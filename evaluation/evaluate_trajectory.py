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
import json
import os
import sys
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_tum(filepath):
    """Read TUM-format file: timestamp tx ty tz qx qy qz qw"""
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
            if ts > 1e15:
                ts *= 1e-9
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            poses[ts] = T
    return poses


def read_euroc_csv(filepath):
    """
    Read EuRoC groundtruth data.csv.
    Format: timestamp[ns], px, py, pz, qw, qx, qy, qz, ...
    Timestamps are converted from nanoseconds to seconds.
    """
    poses = {}
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0]) * 1e-9   # ns -> s
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                qw, qx, qy, qz = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            except ValueError:
                continue  # skip header row
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            poses[ts] = T
    return poses


def read_trajectory(filepath):
    """Auto-detect format: EuRoC data.csv or TUM text file."""
    if os.path.basename(filepath) == "data.csv":
        return read_euroc_csv(filepath)
    return read_tum(filepath)


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
# Main
# ---------------------------------------------------------------------------

def evaluate(gt_poses, frames_file, kf_file, max_diff, rpe_delta):
    """
    ATE  <- keyframes (global consistency of the sparse map)
    RPE  <- frames    (local drift over dense consecutive poses)
    """
    result = {}

    # --- ATE from keyframes ---
    if kf_file:
        print(f"\n[ATE] Keyframes: {kf_file}")
        kf_poses = read_trajectory(kf_file)
        kf_pairs = associate(kf_poses, gt_poses, max_diff=max_diff)
        print(f"  Poses: {len(kf_poses)}  |  Associated: {len(kf_pairs)}")
        if len(kf_pairs) >= 3:
            ate = compute_ate(kf_poses, gt_poses, kf_pairs)
            print_ate(ate)
            result["ate_errors"] = ate["errors"].tolist()
            result["ate_rmse"]   = ate["rmse"]
        else:
            print("  ERROR: too few associations for ATE")
    else:
        print("\n[ATE] No keyframe file provided — skipping ATE")

    # --- RPE from frames ---
    if frames_file:
        print(f"\n[RPE] Frames: {frames_file}")
        f_poses = read_trajectory(frames_file)
        f_pairs = associate(f_poses, gt_poses, max_diff=max_diff)
        print(f"  Poses: {len(f_poses)}  |  Associated: {len(f_pairs)}")
        if len(f_pairs) >= 3:
            rpe = compute_rpe(f_poses, gt_poses, f_pairs, delta=rpe_delta)
            print_rpe(rpe, rpe_delta)
            result["rpe_trans_errors"] = rpe["trans_errors"].tolist()
            result["rpe_trans_rmse"]   = rpe["trans_rmse"]
            result["rpe_rot_errors"]   = rpe["rot_errors"].tolist()
            result["rpe_rot_rmse"]     = rpe["rot_rmse"]
        else:
            print("  ERROR: too few associations for RPE")
    else:
        print("\n[RPE] No frame file provided — skipping RPE")

    return result if result else None


def main():
    parser = argparse.ArgumentParser(
        description="Compute ATE (keyframes) and RPE (frames) for SLAM trajectory evaluation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("groundtruth",  help="Ground truth file (EuRoC data.csv or TUM format)")
    parser.add_argument("--frames",     help="Frame trajectory file — used for RPE")
    parser.add_argument("--keyframes",  help="Keyframe trajectory file — used for ATE")
    parser.add_argument("--max_diff",   type=float, default=0.02,
                        help="Max timestamp difference for association in seconds (default: 0.02)")
    parser.add_argument("--rpe_delta",  type=int, default=1,
                        help="Frame step for RPE computation (default: 1)")
    parser.add_argument("--output_dir", help="Directory to save per-run JSON results for plotting")
    parser.add_argument("--dataset",    help="Dataset ID (e.g. MH01) — used for output filename")
    parser.add_argument("--run",        help="Run label (e.g. folder name) — used for output filename")
    args = parser.parse_args()

    if not args.frames and not args.keyframes:
        parser.error("Provide at least one of --frames or --keyframes")

    print(f"Reading ground truth: {args.groundtruth}")
    gt_poses = read_trajectory(args.groundtruth)
    print(f"  Loaded {len(gt_poses)} poses")

    result = evaluate(gt_poses, args.frames, args.keyframes, args.max_diff, args.rpe_delta)

    if result and args.output_dir and args.dataset and args.run:
        os.makedirs(args.output_dir, exist_ok=True)
        out_path = os.path.join(args.output_dir, f"{args.dataset}_{args.run}.json")
        with open(out_path, "w") as f:
            json.dump({"dataset": args.dataset, "run": args.run, **result}, f)
        print(f"\n  Results saved to: {out_path}")


if __name__ == "__main__":
    main()
