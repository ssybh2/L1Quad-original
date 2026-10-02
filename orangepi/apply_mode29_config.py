#!/usr/bin/env python3
"""Apply a versioned Mode 29 tuning profile to Pixhawk AP_Param storage.

The YAML file is the human-reviewed source of truth. This tool:
- only permits the Mode 29 controller parameters in ALLOWED_PARAMS,
- refuses to write while the vehicle is armed,
- snapshots the current values before changing anything,
- writes parameters one at a time and verifies every readback.
"""

import argparse
from datetime import datetime
import math
from pathlib import Path
import sys
import time

import yaml
from pymavlink import mavutil

DEFAULT_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_2E001F000B51303434333038-if00"

ALLOWED_PARAMS = {
    "GEOCTRL_KPX",
    "GEOCTRL_KPY",
    "GEOCTRL_KPZ",
    "GEOCTRL_KVX",
    "GEOCTRL_KVY",
    "GEOCTRL_KVZ",
    "GEOCTRL_KRX",
    "GEOCTRL_KRY",
    "GEOCTRL_KRZ",
    "GEOCTRL_KOX",
    "GEOCTRL_KOY",
    "GEOCTRL_KOZ",
    "L1ENABLE",
    "ASV",
    "ASOMEGA",
    "CTOFFQ1THRUST",
    "CTOFFQ1MOMENT",
    "CTOFFQ2MOMENT",
    "M29_TKOFF_ALT",
    "M29_TKOFF_T",
    "M29_SETTLE_T",
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("config", help="YAML tuning profile")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source-system", type=int, default=255)
    p.add_argument("--dry-run", action="store_true", help="show changes without writing")
    return p.parse_args()


def collect_parameters(node, output):
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ALLOWED_PARAMS:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{key} must be numeric")
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"{key} must be finite")
                if key in output:
                    raise ValueError(f"duplicate parameter {key}")
                output[key] = value
            else:
                collect_parameters(value, output)
    elif isinstance(node, list):
        for item in node:
            collect_parameters(item, output)


def load_profile(path):
    with path.open("r", encoding="utf-8") as f:
        document = yaml.safe_load(f)
    if not isinstance(document, dict):
        raise ValueError("profile must contain a YAML mapping")

    params = {}
    collect_parameters(document.get("parameters", {}), params)
    if not params:
        raise ValueError("profile contains no supported Mode 29 parameters")

    def find_unknown(node):
        unknown = []
        if isinstance(node, dict):
            for key, value in node.items():
                if isinstance(key, str) and (
                    key.startswith("GEOCTRL_")
                    or key.startswith("CTOFF")
                    or key.startswith("M29_")
                    or key in {"L1ENABLE", "ASV", "ASOMEGA"}
                ) and key not in ALLOWED_PARAMS:
                    unknown.append(key)
                unknown.extend(find_unknown(value))
        elif isinstance(node, list):
            for item in node:
                unknown.extend(find_unknown(item))
        return unknown

    unknown = sorted(set(find_unknown(document.get("parameters", {}))))
    if unknown:
        raise ValueError("unsupported parameter(s): " + ", ".join(unknown))

    if "L1ENABLE" in params and params["L1ENABLE"] not in (0.0, 1.0):
        raise ValueError("L1ENABLE must be 0 or 1")
    if "M29_TKOFF_ALT" in params and not (0.2 <= params["M29_TKOFF_ALT"] <= 5.0):
        raise ValueError("M29_TKOFF_ALT must be in [0.2, 5.0] m")
    if "M29_TKOFF_T" in params and not (1.0 <= params["M29_TKOFF_T"] <= 15.0):
        raise ValueError("M29_TKOFF_T must be in [1.0, 15.0] s")
    if "M29_SETTLE_T" in params and not (0.0 <= params["M29_SETTLE_T"] <= 15.0):
        raise ValueError("M29_SETTLE_T must be in [0.0, 15.0] s")

    return document, params


def connect(args):
    m = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=args.source_system,
        autoreconnect=False,
    )
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No Pixhawk heartbeat on if00")

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        m.close()
        raise RuntimeError("Refusing to change Mode 29 gains while the vehicle is ARMED")

    return m, hb.get_srcSystem(), hb.get_srcComponent()


def request_param(m, target_system, target_component, name, timeout=3.0):
    m.mav.param_request_read_send(
        target_system,
        target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("ascii", errors="ignore")
        pid = str(pid).rstrip("\x00")
        if pid == name:
            return float(msg.param_value)
    return None


def write_param(m, target_system, target_component, name, value):
    m.mav.param_set_send(
        target_system,
        target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("ascii", errors="ignore")
        pid = str(pid).rstrip("\x00")
        if pid == name:
            confirmed = float(msg.param_value)
            if abs(confirmed - value) <= 1e-4:
                return confirmed
            break

    confirmed = request_param(m, target_system, target_component, name)
    if confirmed is None or abs(confirmed - value) > 1e-4:
        raise RuntimeError(
            f"{name}: requested {value}, read back {confirmed}"
        )
    return confirmed


def write_backup(current_values, source_profile):
    backup_dir = Path.home() / "mode29_param_backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = backup_dir / f"mode29_before_apply_{stamp}.yaml"
    payload = {
        "version": 1,
        "profile": {
            "name": f"automatic_backup_{stamp}",
            "source_profile": str(source_profile),
            "created_local": datetime.now().isoformat(timespec="seconds"),
        },
        "parameters": {
            "snapshot": current_values,
        },
    }
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    return path


def main():
    args = parse_args()
    profile_path = Path(args.config).expanduser().resolve()

    if not profile_path.is_file():
        raise SystemExit(f"Config not found: {profile_path}")

    try:
        document, requested = load_profile(profile_path)
        m, target_system, target_component = connect(args)
    except Exception as exc:
        raise SystemExit(f"ERROR: {exc}") from exc

    profile_name = document.get("profile", {}).get("name", profile_path.stem)
    print(f"profile: {profile_name}")
    print(f"connected: sysid={target_system} compid={target_component}")
    print("vehicle: DISARMED")

    current = {}
    for name in requested:
        value = request_param(m, target_system, target_component, name)
        if value is None:
            m.close()
            raise SystemExit(f"ERROR: could not read {name}")
        current[name] = value

    changed = [
        name for name, value in requested.items()
        if abs(current[name] - value) > 1e-4
    ]

    print("\nPlanned values:")
    for name, value in requested.items():
        marker = "*" if name in changed else "="
        print(f"  {marker} {name:18s} {current[name]: .7g} -> {value: .7g}")

    if args.dry_run:
        print(f"\nDRY RUN: {len(changed)} parameter(s) would change")
        m.close()
        return 0

    backup_path = write_backup(current, profile_path)
    print(f"\nbackup: {backup_path}")

    try:
        for name in changed:
            confirmed = write_param(
                m,
                target_system,
                target_component,
                name,
                requested[name],
            )
            print(f"confirmed: {name}={confirmed}")
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        print(
            "A partial write may have occurred. Restore with:\n"
            f"  python3 orangepi/apply_mode29_config.py {backup_path}",
            file=sys.stderr,
        )
        m.close()
        return 2

    print(f"\nSUCCESS: {len(changed)} parameter(s) changed")
    print("No firmware rebuild or reflashing is required.")
    m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
