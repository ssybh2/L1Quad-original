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

    if injected:
        confirmed_injected = [r for r in injected if number(r, "fdi_state", 0) == 2]
        retry = [r for r in injected if number(r, "pair_retry_pending", 0) > 0.5]
        if confirmed_injected:
            estimates = [number(r, "fdi_loss_estimate_pct") for r in confirmed_injected]
            last = confirmed_injected[-1]
            print("Confirmed FDI severity at start/end: {:.2f}% / {:.2f}%".format(
                estimates[0], estimates[-1]))
            print("Confirmed FDI severity range: {:.2f}..{:.2f}%".format(
                min(estimates), max(estimates)))
        if retry:
            causes = {}
            for r in retry:
                reason = r.get("pair_transient_reason", "")
                causes[reason] = causes.get(reason, 0) + 1
            print("Pair severity retry samples:", len(retry))
            print("Pair transient reasons:", causes)
            print("Retry final time: {:.3f}s".format(number(retry[-1], "t")))
        cool = [r for r in rows if number(r, "fdi_cooldown_remaining_s", 0) > 0]
        if cool:
            print("FDI recovery cooldown observed {:.3f}s..{:.3f}s".format(
                number(cool[0], "t"), number(cool[-1], "t")))
        # A correctly latched FDI may remain confirmed briefly after the
        # injected fault ends. This is normal recovery delay, NOT a fresh
        # healthy-motor false confirmation. Track 2->0->2 after the event.
        release_t = number(injected[-1], "t")
        states_after = [r for r in rows if number(r, "t") > release_t + 0.01]
        clear_after_release = False
        reconditions = []
        for r in states_after:
            state = int(number(r, "fdi_state", 0))
            if state != 2:
                clear_after_release = True
            elif clear_after_release:
                if not reconditions:
                    reconditions.append(number(r, "t"))
                clear_after_release = False
        initial_recovery_lag = [
            r for r in states_after
            if number(r, "fdi_state", 0) == 2
            and not reconditions
        ]
        print("FDI fresh confirmations AFTER first clear:", len(reconditions),
              "at", reconditions[:8])
        print("Initial post-release confirmed samples (recovery lag):",
              len(initial_recovery_lag))

    # Distinguish a *committed* mirrored reduction from merely arming the
    # experimental state machine. This helps diagnose delayed re-engagement.
    armed = [r for r in rows if number(r, "pair_controller_armed", 0) > .5]
    if armed:
        print(f"Pair controller armed at: {number(armed[0], 't'):.3f}s")
        applied = [number(r, "pair_mirror_applied_pct", 0.) for r in rows]
        print(f"Maximum ACTUAL opposite derating: {max(applied):.2f}%")
        print(f"Maximum FDI target opposite derating: "
              f"{max(number(r, 'pair_mirror_target_pct', 0.) for r in rows):.2f}%")
        print("Longest continuous full-wrench-feasible windows are gated by "
              "feasible_hold_s; zero mirror means no artificial second failure.")
    retries = max(number(r, "pair_retry_count", 0.) for r in rows)
    rollback = max(number(r, "pair_rollback_count", 0.) for r in rows)
    yaw_backoffs = max(number(r, "pair_yaw_backoff_count", 0.) for r in rows)
    print(f"Temporary pair retries={int(retries)}, "
          f"wrench rollbacks={int(rollback)}, yaw backoffs={int(yaw_backoffs)}")

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
