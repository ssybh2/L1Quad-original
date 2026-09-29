# Softdrone real-airframe configuration

This branch is a real-flight specialization of L1Quad for the user's Softdrone airframe and Pixhawk 6C Mini.

## Airframe constants

- Mass: **1.3854 kg**
- Principal moments of inertia about the ArduPilot body axes:
  - Jx = **0.02218 kg m^2**
  - Jy = **0.02256 kg m^2**
  - Jz = **0.03372 kg m^2**
- Inverse inertia:
  - 1/Jx = **45.085663**
  - 1/Jy = **44.326241**
  - 1/Jz = **29.655991**
- Motor-center geometry for the square X layout:
  - L = **0.28 m** (left-right span, i.e. 2*|y_motor|)
  - D = **0.28 m** (front-rear span, i.e. 2*|x_motor|)

The ArduPilot/L1Quad body frame is **FRD**:

- +X_B: forward / nose
- +Y_B: right
- +Z_B: down

## Motor order

The nonlinear mixer is configured for:

| Output | Physical motor | Body-frame position | Rotation |
|---|---|---|---|
| 1 / w[0] | front-right | (+D/2, +L/2) | CCW |
| 2 / w[1] | rear-left | (-D/2, -L/2) | CCW |
| 3 / w[2] | front-left | (+D/2, -L/2) | CW |
| 4 / w[3] | rear-right | (-D/2, +L/2) | CW |

This ordering matches the sign pattern already used by the original L1Quad mixer, so no mixer permutation is required.

## Static motor/propeller model

The fitted model follows the original L1Quad form

    F(w) = a_F * w^2 + b_F * w
    M(w) = a_M * w^2 + b_M * w

with

    w = (PWM_us - 1000) / 10

and the fitted coefficients are

    a_F = 0.000968094
    b_F = 0.004763730
    a_M = 0.0000107130307
    b_M = 0.000243044484

where F is in newtons and M is in N*m. The primary fit was validated over approximately 1050--1800 us.

## Navigation frame and motion-capture frame

L1Quad obtains position and velocity from ArduPilot AHRS in **NED**:

- +X_N = North / experiment-forward
- +Y_N = East / experiment-right
- +Z_N = Down

The motion-capture world frame for this airframe is defined as:

- +X_M = right
- +Y_M = forward
- +Z_M = up

Therefore the world-frame conversion is

    [x_N]   [ 0  1  0] [x_M]
    [y_N] = [ 1  0  0] [y_M]
    [z_N]   [ 0  0 -1] [z_M]

or, equivalently,

    x_N =  y_M
    y_N =  x_M
    z_N = -z_M

For relative position, subtract the takeoff-origin position in the mocap frame before applying the matrix.

At the intended zero-yaw starting pose:

- body +X_B aligns with mocap +Y_M,
- body +Y_B aligns with mocap +X_M,
- body +Z_B aligns with mocap -Z_M,

so the transformed ArduPilot attitude is identity/yaw=0 at the starting pose.

Do **not** change the L1Quad internal state equations to the mocap convention. Convert mocap measurements to NED/FRD before they enter ArduPilot ExternalNav.

A small conversion helper is provided at:

    tools/softdrone_mocap_frame.py

## Real-hardware build

This branch defaults `REAL_OR_SITL=1`.

Local build:

```bash
git clone --recurse-submodules https://github.com/ssybh2/L1Quad-original.git
cd L1Quad-original
git checkout softdrone

chmod +x installation.sh
./installation.sh

cd ardupilot
. ~/.profile
./waf configure --board Pixhawk6C
./waf copter
```

Expected firmware:

    ardupilot/build/Pixhawk6C/bin/arducopter.apj

Pixhawk 6C Mini uses the Pixhawk6C ArduPilot build target.

## Before any propeller-on test

1. Verify output 1/2/3/4 physically correspond to front-right / rear-left / front-left / rear-right.
2. Verify rotations are CCW / CCW / CW / CW as listed above.
3. Verify the mocap bridge reports (0,0,0) at the takeoff origin.
4. Move the aircraft forward by hand: NED x must increase.
5. Move it right: NED y must increase.
6. Lift it upward: NED z must become negative.
7. With props removed, verify the custom firmware and Mode 29 behavior before flight.
