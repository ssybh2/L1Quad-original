#!/usr/bin/env python3
"""Apply runtime flight-controller tuning parameters to Pixhawk AP_Param storage.

This is the unified Orange Pi tuning tool for the Softdrone experiment.

Supported groups:
- Mode 29 custom geometric/L1 controller parameters.
- ArduCopter attitude/rate parameters used by STABILIZE and other normal
  ArduCopter attitude-controlled modes.

Safety properties:
- writes are allowed only while DISARMED;
- only an explicit parameter whitelist is accepted;
- every requested value is read first;
- the current values are backed up before a write;
- every PARAM_SET is read back and verified;
- no firmware rebuild or reflash is required for these AP_Param values.

Important: the custom Mode 29 controller directly allocates motor commands and
therefore does NOT use the standard ATC_RAT_* / ATC_ANG_* PID gains as its
primary controller. A mixed YAML profile is supported so both controller
families can be versioned and changed from the same Orange Pi workflow.
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

MODE29_PARAMS = {
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
}

ARDUCOPTER_ATTITUDE_PARAMS = {
    "ATC_ANG_RLL_P",
    "ATC_ANG_PIT_P",
    "ATC_ANG_YAW_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_RAT_RLL_FF",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_D",
    "ATC_RAT_PIT_FF",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_FF",
    "ATC_INPUT_TC",
}

ALLOWED_PARAMS = MODE29_PARAMS | ARDUCOPTER_ATTITUDE_PARAMS


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("config", help="YAML tuning profile")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source-system", type=int, default=255)
    p.add_argument("--dry-run", action="store_true", help="show changes without writing")
    return p.parse_args()


def classify(name):
    if name in MODE29_PARAMS:
        return "mode29"
    if name in ARDUCOPTER_ATTITUDE_PARAMS:
        return "arducopter_attitude"
    return "unknown"


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


def find_unknown_controller_params(node):
    unknown = []
    if isinstance(node, dict):
        for key, value in node.items():
            if isinstance(key, str):
                controller_like = (
                    key.startswith("GEOCTRL_")
                    or key.startswith("CTOFF")
                    or key.startswith("ATC_")
                    or key in {"L1ENABLE", "ASV", "ASOMEGA"}
                )
                if controller_like and key not in ALLOWED_PARAMS:
                    unknown.append(key)
            unknown.extend(find_unknown_controller_params(value))
    elif isinstance(node, list):
        for item in node:
            unknown.extend(find_unknown_controller_params(item))
    return unknown


def load_profile(path):
    with path.open("r", encoding="utf-8") as f:
        document = yaml.safe_load(f)

    if not isinstance(document, dict):
        raise ValueError("profile must contain a YAML mapping")

    parameter_tree = document.get("parameters", {})
    requested = {}
    collect_parameters(parameter_tree, requested)

    if not requested:
        raise ValueError("profile contains no supported flight-controller parameters")

    unknown = sorted(set(find_unknown_controller_params(parameter_tree)))
    if unknown:
        raise ValueError("unsupported parameter(s): " + ", ".join(unknown))

    if "L1ENABLE" in requested and requested["L1ENABLE"] not in (0.0, 1.0):
        raise ValueError("L1ENABLE must be 0 or 1")

    return document, requested


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
        raise RuntimeError("Refusing to change controller parameters while the vehicle is ARMED")

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
        raise RuntimeError(f"{name}: requested {value}, read back {confirmed}")

    return confirmed


def write_backup(current_values, source_profile):
    backup_dir = Path.home() / "flight_param_backups"
    backup_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = backup_dir / f"flight_before_apply_{stamp}.yaml"

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

    mode29_count = sum(classify(name) == "mode29" for name in requested)
    atc_count = sum(classify(name) == "arducopter_attitude" for name in requested)
    print(f"requested: mode29={mode29_count}, arducopter_attitude={atc_count}")

    current = {}
    for name in requested:
        value = request_param(m, target_system, target_component, name)
        if value is None:
            m.close()
            raise SystemExit(f"ERROR: could not read {name}")
        current[name] = value

    changed = [
        name
        for name, value in requested.items()
        if abs(current[name] - value) > 1e-4
    ]

    print("\nPlanned values:")
    for name, value in requested.items():
        marker = "*" if name in changed else "="
        group = classify(name)
        print(
            f"  {marker} [{group:20s}] {name:18s} "
            f"{current[name]: .7g} -> {value: .7g}"
        )

    if args.dry_run:
        print(f"\nDRY RUN: {len(changed)} parameter(s) would change")
        print("No firmware rebuild or reflash is required.")
        m.close()
        return 0

    if not changed:
        print("\nSUCCESS: already at requested values")
        print("No firmware rebuild or reflash is required.")
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
            "A partial write may have occurred. Restore while DISARMED with:\n"
            f"  python3 orangepi/apply_flight_config.py {backup_path}",
            file=sys.stderr,
        )
        m.close()
        return 2

    print(f"\nSUCCESS: {len(changed)} parameter(s) changed and verified")
    print("Changes are runtime AP_Param values; no rebuild or reflash is required.")
    m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
