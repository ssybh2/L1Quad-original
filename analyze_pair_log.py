#!/usr/bin/env python3
"""Summarize Mode29 opposite-pair MuJoCo CSV. Python standard library only."""
import argparse
import csv
import math
from pathlib import Path


def number(row, key, default=float("nan")):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_path", type=Path, help="CSV from mode29_mujoco.py")
    args = p.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("Empty flight log")

    injected = [r for r in rows if number(r, "fault_active", 0) > 0.5]
    paired = [r for r in rows if number(r, "pair_active", 0) > 0.5]
    confirmed = [r for r in rows if number(r, "fdi_state", 0) == 2]
    times = [number(r, "t") for r in rows]
    dt = (times[-1]-times[0])/max(1, len(rows)-1)
    source = rows[0].get("pair_source", "unknown")

    print(f"Log: {args.csv_path}")
    print(f"Source: {source}; duration: {times[-1]:.2f}s; rows: {len(rows)}")
    print(f"Fault first active: {number(injected[0], 't'):.3f}s" if injected else "No injected failure")
    print(f"FDI first confirmed: {number(confirmed[0], 't'):.3f}s" if confirmed else "FDI never confirmed")
    if paired:
        start = number(paired[0], "t")
        end = number(paired[-1], "t")
        print(f"Opposite pair engaged: {start:.3f}s; active for {len(paired)*dt:.3f}s")
        print("Paired motors: M{}/M{}, frozen paired loss: {:.1f}%".format(
            int(number(paired[0], "pair_failed_motor", 0)),
            int(number(paired[0], "pair_opposite_motor", 0)),
            number(paired[0], "pair_loss_estimate_pct"),
        ))
        peak_rate = max(abs(number(r, "yaw_rate_rps", 0)) for r in paired)
        print("Peak during pair: |body yaw rate|={:.3f} rad/s ({:.1f} deg/s), XY={:.3f}m, Z={:.3f}m".format(
            peak_rate, math.degrees(peak_rate),
            max(math.hypot(number(r, "x"), number(r, "y")) for r in paired),
            max(abs(number(r, "z")-number(r, "target_z")) for r in paired),
        ))
        cap = number(paired[0], "pair_yaw_cap_deg_s", 0.0)
        target_rate = number(paired[0], "pair_yaw_target_deg_s", 0.0)
        if cap > 0:
            within = sum(
                math.degrees(abs(number(r, "yaw_rate_rps", 0))) <=
                number(r, "pair_yaw_cap_deg_s", cap)
                for r in paired
            )
            saturated = sum(number(r, "pair_yaw_saturated", 0) > 0.5
                            for r in paired)
            print(f"Yaw max speed cap: {cap:.1f} deg/s; signed spin target: {target_rate:.1f} deg/s")
            print(f"Yaw cap compliance: {within}/{len(paired)} samples within cap")
            print(f"Yaw moment allocator saturation: {saturated}/{len(paired)} samples")
    else:
        print("Opposite pair never engaged (check FDI estimate/threshold/guard)")

    stopped = [r for r in rows if r.get("pair_guard_reason")]
    if stopped:
        print(f"Pair disengagement reason: {stopped[-1]['pair_guard_reason']}")

    tail = rows[-1]
    xyz = tuple(number(tail, c) for c in ("x", "y", "z"))
    target = tuple(number(tail, c) for c in ("target_x", "target_y", "target_z"))
    distance = math.dist(xyz, target)
    print(f"Final NED position: {xyz}; target: {target}; error={distance:.3f}m")
    if injected:
        estimates = [number(r, "fdi_loss_estimate_pct") for r in injected]
        truths = [number(r, "fault_loss_pct") for r in injected]
        paired_values = [(a,b) for a,b in zip(estimates,truths)
                         if math.isfinite(a) and math.isfinite(b)]
        if paired_values:
            mae = sum(abs(a-b) for a,b in paired_values)/len(paired_values)
            print(f"FDI injection-relative loss MAE (including unconfirmed samples): {mae:.1f} pp")
            if source == "oracle":
                print("NOTE: This run uses injection truth for opposite pairing;")
                print("      the printed FDI error is diagnostic, not control input.")


if __name__ == "__main__":
    main()
