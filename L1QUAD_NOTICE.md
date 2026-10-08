# L1Quad derivative notice

This software uses or is derived from the L1Quad software developed by the
Department of Mechanical Science and Engineering at the University of Illinois
Urbana-Champaign.

The L1 adaptive augmentation in `mode29_mujoco.py` follows the state predictor,
piecewise-constant uncertainty estimator, and low-pass-filtered adaptive control
structure in `L1AC_customization/ArduCopter/mode_adaptive.cpp` from:

<https://github.com/ssybh2/L1Quad-original>

The upstream L1Quad license permits research, academic, educational, and
personal use subject to its terms. Consult the upstream `License.txt` before
redistribution or other use.
