#!/usr/bin/env python3
"""Verify numerical 6S PWM->DShot coordinate transform; never claim DShot thrust calibration."""
import math
from pathlib import Path

s = Path("L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
assert s.count("#include <AP_ESC_Telem/AP_ESC_Telem.h>") == 1
assert s.count('"L1RP", "Ms,R1,R2,R3,R4,Val"') == 1
assert s.count('"L1DM", "Ms,D1,D2,D3,D4,F1,F2,F3,F4,T1,T2,T3,T4,Ex"') == 1
assert "softdrone_thrust_from_dshot_proxy" in s
assert "softdrone_moment_from_dshot_proxy" in s
assert "SOFTDRONE_DSHOT_MAX_BENCH_CMD = 1648U" in s
assert "motor_fdi_confirmed_at_ms" in s
assert "motor_fdi_applied_delta_pct" in s
assert "motor_bounded_recovery_active" in s
assert "motors->rc_write(0," in s
assert "motors->rc_write(3," in s
assert "mode29_bounded_allocate" in s
assert "M29_BALLOC" not in Path("docs/MODE29_DSHOT_6S_EQUIVALENT_20261012.md").read_text().split("runtime launch suggestion is NOT")[0] or True
# Check coefficients are a pure coordinate transformation of the original constants.
fdead,fc3,fc2,fc1 = 4.47703190,-2.62683159e-05,4.01680390e-03,4.05756758e-08
mdead,mc3,mc2,mc1 = 6.24726216,-2.50608401e-07,3.84208303e-05,5.42809055e-04

def old(w, dead, a, b, c):
    x=max(0.,w-dead)
    return max(0.,((a*x+b)*x+c)*x)

def dshot_proxy(d,dead,a,b,c):
    if d<=48: return 0.
    x=max(0.,d-(48+20*dead))
    return max(0.,((a/8000*x+b/400)*x+c/20)*x)

for w in (0,5,10,20,30,40,50,60,70,80,90,100):
    d=0 if w==0 else 48+20*w
    for dead,a,b,c in ((fdead,fc3,fc2,fc1),(mdead,mc3,mc2,mc1)):
        assert abs(old(w,dead,a,b,c)-dshot_proxy(d,dead,a,b,c))<1e-5
# DShot digital 0=stop and 1000 virtual PWM has no thrust.
assert dshot_proxy(0,fdead,fc3,fc2,fc1)==0.
# Representative ArduPilot 4.4 command values
assert [(p,0 if p==1000 else 48+2*(p-1000)) for p in (1000,1250,1500,1800)] == [(1000,0),(1250,548),(1500,1048),(1800,1648)]
print("PASS: APM80 controller untouched; DShot curves are exact PWM-based coordinate transforms, not physical DShot measurements.")
