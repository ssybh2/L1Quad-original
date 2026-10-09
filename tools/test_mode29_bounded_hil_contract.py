#!/usr/bin/env python3
"""Offline geometry / source-contract checks for the *HIL-only* C++ transition.

These checks do not execute ArduPilot, validate the estimator, establish
real actuator effectiveness, or certify flight safety.
"""
import itertools
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT/"L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text(encoding="utf-8")
HEADER = (ROOT/"L1AC_customization/ArduCopter/mode.h").read_text(encoding="utf-8")
PARAM = (ROOT/"L1AC_customization/ArduCopter/Parameters.cpp").read_text(encoding="utf-8")


def thrust(w):
    x=max(0.0,w-4.47703190)
    return max(0.0, ((-2.62683159e-5*x+4.01680390e-3)*x+
                     4.05756758e-8)*x)


def moment(w):
    x=max(0.0,w-6.24726216)
    return max(0.0, ((-2.50608401e-7*x+3.84208303e-5)*x+
                     5.42809055e-4)*x)


def w_from_f(f):
    if f<=0: return 0.0
    lo,hi=0.0,100.0
    for _ in range(30):
        mid=(lo+hi)*0.5
        if thrust(mid)<f: lo=mid
        else: hi=mid
    return (lo+hi)*0.5


def exact_primary(cmd, pwm_caps):
    # Cap-limited four-rotor nullspace: f=f0+s*[1,1,-1,-1].
    F,mx,my=cmd[:3]
    cap=[thrust(w) for w in pwm_caps]
    if F<-1e-6 or F>sum(cap)+1e-6: return None
    L=D=0.28
    base=[F/4-mx/(2*L)+my/(2*D),
          F/4+mx/(2*L)-my/(2*D),
          F/4+mx/(2*L)+my/(2*D),
          F/4-mx/(2*L)-my/(2*D)]
    signs=[1,1,-1,-1]
    lo,hi=-float("inf"),float("inf")
    for b,k,c in zip(base,signs,cap):
        a=-b/k
        z=(c-b)/k
        lo=max(lo,min(a,z))
        hi=min(hi,max(a,z))
    if lo>hi+1e-5: return None
    return base,(lo,hi),cap


def yaw_interval(cmd,pwm_caps):
    data=exact_primary(cmd,pwm_caps)
    if data is None: return None
    base,(lo,hi),cap=data
    sign=[1,1,-1,-1]
    def yaw(s):
        f=[min(c,max(0,b+s*k)) for b,k,c in zip(base,sign,cap)]
        return sum(k*moment(w_from_f(v)) for k,v in zip(sign,f))
    a,b=yaw(lo),yaw(hi)
    return min(a,b),max(a,b)


def check_source_contract():
    assert 'GSCALAR(m29_balloc_en, "M29_BALLOC", 0)' in PARAM
    assert 'void ModeAdaptive::update_bounded_pair_mode' in CPP
    assert 'mode29_primary_yaw_interval' in CPP
    assert 'constexpr uint32_t hold_ms=150U' in CPP
    assert 'constexpr uint32_t retry_ms=300U' in CPP
    assert 'constexpr float ramp_pp_s=70.0f' in CPP
    assert 'worsens_braking' in CPP
    assert 'motor_pair_retry_count++' in CPP
    assert 'motor_pair_loss_pct=MIN(motor_pair_loss_pct,' in CPP
    assert 'motor_bounded_injected_loss_pct-100.0f*0.0025f' in CPP
    assert 'motor_bounded_recovery_cooldown_until_ms' in CPP
    assert 'motor_bounded_yaw_unbrakeable' in CPP
    assert 'motor_bounded_fault_seen_this_run' in CPP
    assert 'AP::logger().Write("L1PB"' in CPP
    assert 'AP::logger().Write("L1BA"' in CPP
    assert 'motor_pair_target_loss_pct' in HEADER
    assert 'motor_pwm_cap_baseline=motor_pwm_last_sent;' in CPP
    assert 'softdrone_thrust_from_w(pwm_caps[i])' in CPP
    assert 'motorPWM[i]=MIN(motorPWM[i],injected_cap_pwm[i]);' in CPP
    assert 'AP::logger().Write("L1PC"' in CPP
    # A 150-ms hold must not clear the same timer in the waiting branch.
    snippet=CPP.split('if (now-motor_pair_feasible_since_ms<hold_ms) {',1)[1].split('return;',1)[0]
    assert 'withdraw(' not in snippet
    assert 'motor_pair_feasible_since_ms=0' not in snippet


def check_continuous_geometry():
    for motor in range(4):
        for loss in (0,0.001,7.3,43.5,60,70,80,84.22,90,96.5,100):
            pwm_caps=[100.0]*4
            pwm_caps[motor]=40.0*(1.0-loss/100.0)
            for F in (0,7.4,10.29,13.56,19.0):
                cmd=[F,-.083,.007,0]
                data=exact_primary(cmd,pwm_caps)
                if data is None: continue
                b,(lo,hi),cap=data
                for s in (lo,(lo+hi)/2,hi):
                    f=[bi+s*ki for bi,ki in zip(b,(1,1,-1,-1))]
                    assert abs(sum(f)-F)<1e-4
                    assert all(-1e-4<=fi<=ci+1e-4 for fi,ci in zip(f,cap))
                    mx=.14*(-f[0]+f[1]+f[2]-f[3])
                    my=.14*(f[0]-f[1]+f[2]-f[3])
                    assert abs(mx-cmd[1])<1e-4 and abs(my-cmd[2])<1e-4


def check_high_loss_and_braking():
    # 80% PWM cap at a previous w=40 produces only w<=8.
    wcap=40*.2
    assert wcap==8
    assert thrust(wcap)<thrust(40)*.2
    caps=[wcap,100,100,100]
    assert exact_primary([10.474,-1.904,1.779,0],caps) is None
    # Pair feasibility and yaw authority are tested under capped PWM.
    # The previous nonzero moment is ALSO physically infeasible with
    # near-dead PWM capped M1: rejection is expected and correct.
    stressed=yaw_interval([10.29,-.083,.007,0],[wcap,100,100,100])
    assert stressed is None
    # Hover with zero roll/pitch can remain feasible by allocating the
    # primary wrench on the two non-capped motors, without yaw guarantee.
    no_mirror=yaw_interval([10.29,0,0,0],[wcap,100,100,100])
    paired=yaw_interval([10.29,0,0,0],[wcap,wcap,100,100])
    assert no_mirror is not None
    assert paired is not None
    assert all(math.isfinite(x) for x in no_mirror+paired)
    print("HIL PWM-cap geometry: infeasible wrench refused;",
          "zero RP yaw authority before/after mirror",no_mirror,paired)


if __name__=="__main__":
    check_source_contract()
    check_continuous_geometry()
    check_high_loss_and_braking()
    print("PASS: HIL PWM-cap source contracts and 4-motor feasibility")
