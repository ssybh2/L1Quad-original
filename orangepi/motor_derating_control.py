#!/usr/bin/env python3
"""Orange Pi runtime motor-derating controller for Softdrone Mode 29.

Uses MAVLink RC_CHANNELS_OVERRIDE on RC9..RC12 only:
  RC9  : enable
  RC10 : motor selector
  RC11 : loss percentage
  RC12 : yaw-free mode

Physical pilot channels 1..8 are never overridden.
"""

import argparse
import os
import signal
import sys
import time

# RC_CHANNELS_OVERRIDE channels 9..18 are MAVLink2 extension fields.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

DEFAULT_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_2E001F000B51303434333038-if00"
MODE_ADAPTIVE = 29
IGNORE_1_8 = 65535
RELEASE_9_18 = 65534


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source-system", type=int, default=255)
    sub = p.add_subparsers(dest="command", required=True)

    en = sub.add_parser("enable")
    en.add_argument("--motor", type=int, choices=(1, 2, 3, 4), required=True)
    en.add_argument("--loss", type=float, required=True, help="thrust-effectiveness loss percent, 0 <= loss <= 100")
    en.add_argument("--duration", type=float, default=5.0, help="seconds; 0 means until Ctrl+C")
    en.add_argument("--rate", type=float, default=10.0)
    en.add_argument("--keep-yaw", action="store_true", help="keep fixed-yaw control instead of requesting yaw-free mode")
    en.add_argument(
        "--blind",
        action="store_true",
        help="blind-FDI experiment: injector keeps yaw enabled; onboard FDI must detect the severe fault and release yaw automatically",
    )
    en.add_argument("--wait-mode29", type=float, default=60.0, help="seconds to wait for armed Mode 29")

    sub.add_parser("disable")
    sub.add_parser("status")
    return p.parse_args()


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
    return m, hb.get_srcSystem(), hb.get_srcComponent()


def request_param(m, target_system, target_component, name, timeout=2.0):
    m.mav.param_request_read_send(
        target_system,
        target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("ascii", errors="ignore")
        pid = str(pid).rstrip("\x00")
        if pid == name:
            return float(msg.param_value)
    return None


def send_override(m, target_system, target_component, rc9, rc10, rc11, rc12):
    m.mav.rc_channels_override_send(
        target_system,
        target_component,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        IGNORE_1_8,
        rc9,
        rc10,
        rc11,
        rc12,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def send_release(m, target_system, target_component):
    for _ in range(5):
        send_override(
            m,
            target_system,
            target_component,
            RELEASE_9_18,
            RELEASE_9_18,
            RELEASE_9_18,
            RELEASE_9_18,
        )
        time.sleep(0.05)


def heartbeat_state(m, timeout=1.0):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        return None
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return armed, int(hb.custom_mode)


def wait_for_armed_mode29(m, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = heartbeat_state(m, timeout=1.0)
        if state is None:
            continue
        armed, mode = state
        print(f"heartbeat: armed={armed} custom_mode={mode}")
        if armed and mode == MODE_ADAPTIVE:
            return True
    return False


def motor_selector_pwm(motor):
    return {1: 1125, 2: 1375, 3: 1625, 4: 1875}[motor]


def loss_to_pwm(loss):
    return int(round(1000.0 + (loss / 100.0) * 1000.0))


def main():
    args = parse_args()
    m, target_system, target_component = connect(args)
    print(f"connected: sysid={target_system} compid={target_component}")

    if args.command == "status":
        for name in ("SYSID_MYGCS", "RC_OVERRIDE_TIME", "FLTMODE_CH", "RC9_OPTION", "RC10_OPTION", "RC11_OPTION", "RC12_OPTION", "TRAJINDEX", "LANDFLAG", "L1ENABLE", "M29_TKOFF_ALT", "M29_TKOFF_T", "M29_SETTLE_T", "M29_GS_MODE", "M29_PAIR_EN"):
            value = request_param(m, target_system, target_component, name)
            print(f"{name}={value}")
        return 0

    if args.command == "disable":
        send_release(m, target_system, target_component)
        print("RC9..RC12 overrides released")
        return 0

    if not (0.0 <= args.loss <= 100.0):
        raise SystemExit("--loss must be >=0 and <=100")

    sysid_mygcs = request_param(m, target_system, target_component, "SYSID_MYGCS")
    if sysid_mygcs is not None and int(round(sysid_mygcs)) != args.source_system:
        raise SystemExit(
            f"SYSID_MYGCS={sysid_mygcs:.0f} but --source-system={args.source_system}; "
            "ArduPilot will ignore RC overrides from the wrong MAVLink system id"
        )

    override_timeout = request_param(m, target_system, target_component, "RC_OVERRIDE_TIME")
    if override_timeout is None:
        raise SystemExit("Could not read RC_OVERRIDE_TIME")
    if override_timeout <= 0.0 or override_timeout > 0.6:
        raise SystemExit(
            f"RC_OVERRIDE_TIME={override_timeout}; set it to 0.5 s before enabling derating"
        )

    trajindex = request_param(m, target_system, target_component, "TRAJINDEX")
    landflag = request_param(m, target_system, target_component, "LANDFLAG")
    takeoff_time = request_param(m, target_system, target_component, "M29_TKOFF_T")
    settle_time = request_param(m, target_system, target_component, "M29_SETTLE_T")
    if trajindex is None or int(round(trajindex)) != 0:
        raise SystemExit(f"TRAJINDEX must be 0 for the hover experiment; got {trajindex}")
    if landflag is None or int(round(landflag)) != 0:
        raise SystemExit(f"LANDFLAG must be 0; got {landflag}")

    if takeoff_time is not None and settle_time is not None:
        print(
            f"firmware fault gate: active only after "
            f"{takeoff_time + settle_time:.1f}s in Mode29 "
            f"(takeoff={takeoff_time:.1f}s + settle={settle_time:.1f}s)"
        )

    print("waiting for ARMED + Mode 29...")
    if not wait_for_armed_mode29(m, args.wait_mode29):
        raise SystemExit("Timed out waiting for armed Mode 29")

    rc9 = 2000
    rc10 = motor_selector_pwm(args.motor)
    rc11 = loss_to_pwm(args.loss)
    rc12 = 1000 if (args.keep_yaw or args.blind) else 2000
    period = 1.0 / max(1.0, args.rate)

    print(
        f"starting: motor={args.motor} loss={args.loss:.1f}% "
        f"yaw_free_requested={not (args.keep_yaw or args.blind)} "
        f"blind_fdi={args.blind} duration={args.duration}s"
    )

    stop = False

    def on_signal(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    start = time.monotonic()
    last_hb_check = 0.0
    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0.0 and now - start >= args.duration:
                break

            send_override(m, target_system, target_component, rc9, rc10, rc11, rc12)

            if now - last_hb_check >= 0.5:
                last_hb_check = now
                hb = m.recv_match(type="HEARTBEAT", blocking=False)
                if hb is not None:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode = int(hb.custom_mode)
                    if not armed or mode != MODE_ADAPTIVE:
                        print(f"stopping: armed={armed} custom_mode={mode}")
                        break

            time.sleep(period)
    finally:
        # First request disable, then release all four override channels.
        for _ in range(2):
            send_override(m, target_system, target_component, 1000, rc10, 1000, 1000)
            time.sleep(0.05)
        send_release(m, target_system, target_component)
        m.close()

    print("motor derating command cleared")
    return 0


if __name__ == "__main__":
    sys.exit(main())
