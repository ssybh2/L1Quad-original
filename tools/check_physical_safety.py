#!/usr/bin/env python3
"""Strict physical acceptance criteria, not merely 'simulation did not crash'.

Does not treat any particular injected fault percentage as exceptional.
The job should remain RED when the simulated airframe cannot track safely.
"""
import argparse
import csv
import math


def check(csv_path, duration, max_error):
    with open(csv_path, encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise AssertionError("no data (simulator stopped before logging)")
    first, last = rows[0], rows[-1]
    dt = float(rows[1]["t"])-float(first["t"]) if len(rows)>1 else 0.0
    last_time = float(last["t"])
    assert last_time >= duration-max(0.01, 3*dt), (
        f"EARLY STOP: last sample t={last_time:.4f}s but requested {duration}s"
    )
    bad = []
    maximum = (0.0, 0.0)
    for r in rows:
        error = math.dist(
            [float(r[x]) for x in ("x","y","z")],
            [float(r[x]) for x in ("target_x","target_y","target_z")]
        )
        assert math.isfinite(error), "non-finite position state"
        if error > maximum[1]:
            maximum = (float(r["t"]), error)
        if error > max_error:
            bad.append(float(r["t"]))
        if str(r.get("allocator_mode", "")).startswith(
            ("fdi_effectiveness_aware", "fdi_candidate_protection",
             "paired_deferred_single_fault", "paired_unachievable_single_fault")
        ):
            predicted = float(r.get("alloc_collective_predicted_n", "nan"))
            achievable = float(r.get("alloc_collective_achievable_n", "nan"))
            assert math.isfinite(predicted) and math.isfinite(achievable)
            assert abs(predicted-achievable) <= 0.02, (
                "bounded allocator did not conserve reachable collective at t="
                +r["t"]
            )
        if int(float(r.get("fdi_state", "0"))) == 2:
            est = float(r.get("fdi_loss_estimate_pct", "nan"))
            assert math.isfinite(est) and 0.0 <= est <= 100.0
            instant = float(r.get("fdi_loss_instant_pct", "nan"))
            assert math.isfinite(instant) and abs(instant) <= 100.001, (
                "FDI trusted instantaneous estimate exceeded physical bounds"
            )
    assert not bad, (
        f"POSITION SAFETY FAIL: {len(bad)} samples >{max_error:.3f}m "
        f"(first t={bad[0]:.3f}s, peak={maximum[1]:.3f}m at {maximum[0]:.3f}s)"
    )
    print(f"PHYSICAL PASS: duration={duration:g}s, maximum 3D error "
          f"{maximum[1]:.4f}m at t={maximum[0]:.3f}s, "
          "collective bounded and FDI estimates finite")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("csv_path")
    p.add_argument("--duration", type=float, required=True)
    p.add_argument("--max-error", type=float, default=.5)
    args = p.parse_args()
    check(args.csv_path, args.duration, args.max_error)
