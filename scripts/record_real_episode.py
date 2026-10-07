#!/usr/bin/env python3
"""Capture a timed real-Go2 episode from rl_real_go2's existing debug CSV.

This script only reads logs. Start it before commanding forward motion; it
begins when obs_cmd_x reaches the requested command threshold.
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--policy", default="benchmarkv8.4")
    parser.add_argument("--command-x", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--source", type=Path, help="Use this live policy CSV instead of discovering the newest one")
    parser.add_argument("--output", type=Path, help="Output directory; defaults to a timestamped directory under debug_logs")
    args = parser.parse_args()
    if args.duration <= 0 or args.timeout <= 0 or args.command_x <= 0:
        parser.error("duration, timeout, and command-x must be positive")

    debug_dir = args.root / "debug_logs"
    started_wall = time.time()
    deadline = time.monotonic() + args.timeout
    pattern = f"real_policy_debug_{args.policy}_*.csv"
    source = args.source
    while source is None or not source.is_file():
        candidates = [p for p in debug_dir.glob(pattern) if p.stat().st_mtime >= started_wall - 5]
        if candidates:
            source = max(candidates, key=lambda p: p.stat().st_mtime)
            break
        if time.monotonic() >= deadline:
            raise TimeoutError(f"No current policy CSV found in {debug_dir} matching {pattern}")
        time.sleep(0.2)

    output = args.output or debug_dir / f"real_episode_{args.policy}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output.mkdir(parents=True, exist_ok=False)
    print(f"Reading {source}; waiting for obs_cmd_x >= {args.command_x:.2f}", flush=True)
    with source.open("r", newline="") as input_file, (output / "episode.csv").open("w", newline="") as output_file:
        header = input_file.readline().rstrip("\r\n")
        fields = next(csv.reader([header]))
        if "obs_cmd_x" not in fields or "wall_time_s" not in fields:
            raise ValueError("Source CSV lacks obs_cmd_x or wall_time_s")
        writer = csv.writer(output_file)
        writer.writerow(fields)
        input_file.seek(0, os.SEEK_END)
        first_time = None
        last_time = None
        first_zero_gain_time = None
        samples = 0
        zero_gain_samples = 0
        while time.monotonic() < deadline:
            position = input_file.tell()
            line = input_file.readline()
            if not line:
                time.sleep(0.05)
                continue
            if not line.endswith("\n"):
                input_file.seek(position)
                time.sleep(0.05)
                continue
            values = next(csv.reader([line]))
            if len(values) != len(fields):
                continue
            row = dict(zip(fields, values))
            stamp = number(row, "wall_time_s")
            if first_time is None:
                if number(row, "obs_cmd_x") < args.command_x - 0.05:
                    continue
                first_time = stamp
                print(f"Episode started at wall_time_s={first_time:.3f}", flush=True)
            if stamp - first_time >= args.duration:
                break
            writer.writerow(values)
            samples += 1
            last_time = stamp
            if number(row, "lowcmd_kp_RR_calf_joint") < 0.01 and number(row, "lowcmd_kd_RR_calf_joint") < 0.01:
                zero_gain_samples += 1
                if first_zero_gain_time is None:
                    first_zero_gain_time = stamp
            if samples % 25 == 0:
                output_file.flush()
        else:
            print("Capture timed out before a full episode was recorded", flush=True)

    metadata = {
        "source_csv": str(source.resolve()),
        "policy_name_requested": args.policy,
        "command_x_threshold_m_s": args.command_x - 0.05,
        "requested_duration_s": args.duration,
        "first_wall_time_s": first_time,
        "last_wall_time_s": last_time,
        "samples": samples,
        "rr_calf_zero_kp_kd_samples": zero_gain_samples,
        "first_rr_calf_zero_kp_kd_s_after_start": None if first_zero_gain_time is None else first_zero_gain_time - first_time,
        "note": "Read-only capture; does not command motion or apply a fault. lin_vel may be unavailable on hardware.",
    }
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Saved {samples} rows to {output}", flush=True)
    if first_time is None or last_time is None or last_time - first_time < args.duration - 0.5:
        raise RuntimeError("Incomplete episode; inspect metadata.json and the source policy log")


if __name__ == "__main__":
    main()
