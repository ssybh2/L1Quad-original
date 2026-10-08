#!/usr/bin/env python3
"""CI acceptance checks for the post-fix blind FDI / opposite-pair MuJoCo trace."""
import argparse
import csv
import math


def numeric(row, key):
    return float(row.get(key, "nan"))


def validate(filename, commanded_loss):
    with open(filename, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise AssertionError("no recorded simulation samples")

    faults = [row for row in rows if numeric(row, "fault_active") > 0.5]
    assert faults, "loss never injected"
    assert abs(numeric(faults[0], "fault_loss_pct") - commanded_loss) < 1e-5
    active = [row for row in rows if numeric(row, "pair_active") > 0.5]
    identified = [row for row in faults
                  if numeric(row, "fdi_state") == 2
                  and numeric(row, "fdi_motor") == 1]
    assert identified, "blind FDI did not identify M1"

    if commanded_loss in (60, 70):
        assert len(active) > 400, (
            "previous transient-bias gate incorrectly prevents long-lived pairing"
        )
        for row in active:
            assert int(numeric(row, "pair_failed_motor")) == 1
            assert int(numeric(row, "pair_opposite_motor")) == 2
            cap = numeric(row, "pair_yaw_cap_deg_s")
            assert math.degrees(abs(numeric(row, "yaw_rate_rps"))) <= cap + 0.2, (
                "actuator yaw speed beyond configured cap in tested scenario"
            )
    elif commanded_loss == 80:
        reasons = {row["pair_guard_reason"] for row in rows
                   if row.get("pair_guard_reason")}
        assert active or any(
            "primary authority infeasible" in msg for msg in reasons
        ), "80% case neither engages safely nor records physical infeasibility"

    after_fault_end = [row for row in rows
                       if numeric(row, "t") > numeric(faults[-1], "t") + 0.01]
    assert after_fault_end, "did not simulate recovery period"
    has_cleared = False
    re_confirm_count = 0
    for row in after_fault_end:
        if numeric(row, "fdi_state") != 2:
            has_cleared = True
        elif has_cleared:
            re_confirm_count += 1
    assert re_confirm_count == 0, "healthy motor falsely re-confirmed after cooldown"
    final = rows[-1]
    error = math.dist(
        [numeric(final, key) for key in ("x", "y", "z")],
        [numeric(final, key) for key in ("target_x", "target_y", "target_z")],
    )
    assert math.isfinite(error), "simulation ended in non-finite position"
    print(
        f"PASS: fault={commanded_loss:g}%, ID=M1, active_pair_samples={len(active)},"
        f" repeat_false_confirmations={re_confirm_count}, final_error={error:.3f}m"
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--loss", required=True, type=float)
    args = ap.parse_args()
    validate(args.path, args.loss)
