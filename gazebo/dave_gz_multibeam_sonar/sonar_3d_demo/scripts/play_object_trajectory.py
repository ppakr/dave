#!/usr/bin/env python3
"""
Play back an object_top trajectory CSV by calling Gazebo's /world/<w>/set_pose.

This is the runtime counterpart to extract_object_poses.py in sonar_camera_logger.
It takes a per-bag CSV (t_ns, t_rel_s, x, y, z, qx, qy, qz, qw, in sonar frame),
adds the sim-world offset for the sonar, drops obvious ArUco pose-flip glitches
with an 8·MAD filter, and republishes each pose as a Gazebo set_pose request at
the recorded inter-frame intervals.

Why a script instead of a ROS node:
  - No ROS-side state is needed; the only outputs are gz service calls.
  - `gz service` already speaks the Gazebo Sim protocol fluently; reusing it
    avoids depending on gz-transport python bindings being installed.

Usage:
  python3 play_object_trajectory.py \
      --csv  /media/aki/2C76C6780AEDB4DB1/wl_wetlab_apr22_processed/object_poses/apr22_petg_circle_moving.csv \
      --model_name circle_petg \
      --world default \
      --sensor_x -0.95 --sensor_y 0.0 --sensor_z 0.5 \
      --rate 10
"""

import argparse
import csv
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def reject_outliers(xyz: np.ndarray, mad_scale: float = 8.0) -> np.ndarray:
    med = np.median(xyz, axis=0)
    mad = np.median(np.abs(xyz - med), axis=0) + 1e-6
    return np.all(np.abs(xyz - med) < mad_scale * mad, axis=1)


def load_csv(path: Path):
    t_rel = []
    xyz = []
    quat = []
    with path.open() as f:
        for row in csv.DictReader(f):
            t_rel.append(float(row["t_rel_s"]))
            xyz.append([float(row["x"]), float(row["y"]), float(row["z"])])
            quat.append([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])])
    return np.array(t_rel), np.array(xyz), np.array(quat)


def gz_set_pose(world: str, name: str, x, y, z, qx, qy, qz, qw, timeout_ms: int = 200):
    """Call /world/<w>/set_pose synchronously via the gz CLI.

    Returns True on success, False on failure (timeout, service missing, etc).
    """
    req = (
        f'name: "{name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        print("ERROR: 'gz' CLI not found in PATH.", file=sys.stderr)
        return False
    return out.returncode == 0 and "true" in (out.stdout or "").lower()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", required=True, help="Per-bag CSV from extract_object_poses.py")
    ap.add_argument("--model_name", required=True,
                    help="Gazebo entity name to drive (same name passed to ros_gz_sim create).")
    ap.add_argument("--world", default="default")
    ap.add_argument("--sensor_x", type=float, default=0.0)
    ap.add_argument("--sensor_y", type=float, default=0.0)
    ap.add_argument("--sensor_z", type=float, default=0.0)
    ap.add_argument("--rate", type=float, default=10.0,
                    help="Target set_pose rate in Hz. CSV is downsampled to this.")
    ap.add_argument("--use_csv_orientation", action="store_true",
                    help="Use rotation from CSV. Default keeps spawn rotation (identity here).")
    ap.add_argument("--qx", type=float, default=0.0,
                    help="Fixed quaternion x when --use_csv_orientation is off.")
    ap.add_argument("--qy", type=float, default=0.0)
    ap.add_argument("--qz", type=float, default=0.0)
    ap.add_argument("--qw", type=float, default=1.0)
    ap.add_argument("--time_rate", type=float, default=1.0,
                    help="Playback speed multiplier (>1 = faster than real time).")
    ap.add_argument("--startup_delay_s", type=float, default=2.0,
                    help="Wait this long before the first set_pose (lets the spawn settle).")
    args = ap.parse_args()

    if shutil.which("gz") is None:
        print("ERROR: 'gz' CLI not on PATH. Source the Gazebo env first.", file=sys.stderr)
        sys.exit(2)

    csv_path = Path(args.csv).expanduser().resolve()
    if not csv_path.is_file():
        print(f"ERROR: CSV not found: {csv_path}", file=sys.stderr)
        sys.exit(2)

    t_rel, xyz, quat = load_csv(csv_path)
    if len(t_rel) == 0:
        print("ERROR: CSV is empty.", file=sys.stderr)
        sys.exit(2)

    keep = reject_outliers(xyz)
    t_rel, xyz, quat = t_rel[keep], xyz[keep], quat[keep]
    print(f"Loaded {len(t_rel)} inlier frames "
          f"(from {len(keep)} total, dropped {(~keep).sum()} outliers).")

    # Downsample to target rate.
    if args.rate > 0:
        keep_idx = [0]
        last_t = t_rel[0]
        period = 1.0 / args.rate
        for i in range(1, len(t_rel)):
            if t_rel[i] - last_t >= period:
                keep_idx.append(i)
                last_t = t_rel[i]
        t_rel = t_rel[keep_idx]
        xyz = xyz[keep_idx]
        quat = quat[keep_idx]
    print(f"Downsampled to {len(t_rel)} frames at ~{args.rate} Hz.")

    # Apply sim_world translation.
    xyz = xyz + np.array([args.sensor_x, args.sensor_y, args.sensor_z])

    if args.startup_delay_s > 0:
        print(f"Waiting {args.startup_delay_s}s before first set_pose ...")
        time.sleep(args.startup_delay_s)

    t0_wall = time.monotonic()
    t0_csv = t_rel[0]
    failures = 0
    for i in range(len(t_rel)):
        target_wall = t0_wall + (t_rel[i] - t0_csv) / max(args.time_rate, 1e-6)
        sleep_for = target_wall - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

        if args.use_csv_orientation:
            qx, qy, qz, qw = quat[i]
        else:
            qx, qy, qz, qw = args.qx, args.qy, args.qz, args.qw

        ok = gz_set_pose(
            args.world, args.model_name,
            xyz[i, 0], xyz[i, 1], xyz[i, 2],
            qx, qy, qz, qw,
        )
        if not ok:
            failures += 1
            if failures == 1:
                print("WARN: first set_pose failed — is the model spawned yet?", file=sys.stderr)

    print(f"Playback done. {len(t_rel) - failures}/{len(t_rel)} set_pose calls succeeded.")


if __name__ == "__main__":
    main()
